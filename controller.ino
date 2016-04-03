#include <math.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Bounce2.h>
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_SSD1306.h>
#include <PString.h>

#define THRESHOLD_MIN 50
#define THRESHOLD_MAX 99
#define THRESHOLD_MID (THRESHOLD_MIN + ((THRESHOLD_MAX - THRESHOLD_MIN) / 2))

#define HIGHER_TEMP 2.0 //+threshold
#define HIGH_HUMIDITY 75.0

#define TEMP_HYSTERESIS 1.75
#define HUMIDITY_HYSTERESIS 5.0

#define OLED_ADDRESS 0x3C
#define OLED_HEADER_HEIGHT 16

#define DOOROUTPIN 11
#define DOORINPIN 12

#define DHTTYPE DHT22
#define DHTPIN 14

#define TABLEFAN 0
#define DESKFAN 1
#define DEHUMIDIFIER 2
#define DOORFAN 3

#define TOUCH_THRESHOLD 1900

#define ICON_WIDTH 16
#define ICON_HEIGHT OLED_HEADER_HEIGHT

static const unsigned char PROGMEM fan_icon[] = {
  B00000111, B11100000,
  B00001101, B10110000,
  B00011011, B11011000,
  B00110011, B11001100,
  B00100011, B11000100,
  B00100001, B10000100,
  B00100111, B11100100,
  B00101111, B11110100,
  B00111110, B01111100,
  B00011100, B00111000,
  B00001100, B00110000,
  B00000111, B11100000,
  B00000001, B10000000,
  B00000011, B11000000,
  B00011111, B11111000,
  B00011111, B11111000
};

static const unsigned char PROGMEM water_icon[] = {
  B00000000, B00000000,
  B00000001, B10000000,
  B00000011, B11000000,
  B00000011, B11000000,
  B00000111, B11100000,
  B00001111, B11110000,
  B00001111, B11110000,
  B00011111, B11111000,
  B00011111, B11111000,
  B00011111, B11111000,
  B00011111, B11011000,
  B00011111, B10011000,
  B00001111, B00110000,
  B00001111, B11110000,
  B00000011, B11000000,
  B00000000, B00000000
};

const char* relay_identifiers = "TKHD";
const int relay_pins[] = {7, 8, 9, 10};
const unsigned char* relay_icons[] = {fan_icon, fan_icon, water_icon, fan_icon};

const int touch_pins[] = {15, 16};
const float touch_pin_values[] = { -0.5, 0.5};

bool display_stale = true;

ThreadController controller;

template <int num, typename _Tp>
class TouchSensorThread : public Thread {
  public:
    struct touchSensorData {
      bool active;
      int pin;
      uint32_t time, start;

      _Tp value;

      touchSensorData() : active(false), time(0), start(0) {}
    };

    const int count;

    bool any_active;

    touchSensorData sensors[num];

    TouchSensorThread(const int pins[num], const _Tp values[num]) : count(num) {
      for (int i = 0; i < num; ++i) {
        sensors[i].pin = pins[i];
        sensors[i].value = values[i];
      }
    }

    inline operator bool() const {
      return any_active;
    }

    /*
      So this basically runs in 2(n-1) time, which is pretty nice.
    */
    const touchSensorData* const getLastActive() const {
      int i, p;

      //Since all sensors update at the same time, it's not possible for the last updated sensor to be not active while another is active
      for (i = 1, p = 0; i < num; ++i) {
        if (sensors[i].time > sensors[p].time) {
          p = i;
        }
      }

      for (i = 0; i < num; ++i) {
        if (i == p) continue; //don't bother comparing the same sensor

        const touchSensorData &cur = sensors[i], &prev = sensors[p];

        //If the two times are the exact same, then both sensors are either active or not. In which case, the one that was activated most recently takes priority
        if (cur.time == prev.time && cur.start > prev.start) {
          p = i;
        }
      }

      return &sensors[p];
    }

    void run() {
      uint32_t now = millis();

      any_active = false;

      for (int i = 0; i < num; ++i) {
        touchSensorData& sensor = sensors[i];

        int reading = touchRead(sensor.pin);

        bool active = reading > TOUCH_THRESHOLD;

        if (active) {
          if (!sensor.active) {
            sensor.start = now;
          }

          sensor.time = now;
        }

        sensor.active = active;

        any_active |= active;
      }

      runned();
    }
};

class DoorSensorThread : public Thread {
  public:
    bool is_open;

    int pin, ground_pin, rate;

    Bounce bouncer;

    DoorSensorThread(int p, int r = 50, int gp = -1) : Thread(), pin(p), ground_pin(gp), rate(r) {
    }

    void setup() {
      if (ground_pin != -1) {
        pinMode(ground_pin, OUTPUT);
        digitalWrite(ground_pin, LOW);
      }

      pinMode(pin, INPUT_PULLUP);

      bouncer.attach(pin);
      bouncer.interval(rate);
    }

    void run() {
      bouncer.update();

      bool now_open = bouncer.read();

      if (now_open != is_open) {
        display_stale = true;
      }

      is_open = now_open;

      runned();
    }
};

class DHTSensorThread : public Thread {
  public:
    float t, h, hi;
    bool error, paused;

    DHT sensor;

    DHTSensorThread(uint8_t pin, uint8_t type) : Thread(), t(0.0), h(0.0), hi(0.0), error(false), paused(false), sensor(pin, type) {
    }

    inline void setup() {
      sensor.begin();
    }

    void run() {
      if (!paused) {
        h = sensor.readHumidity();
        t = sensor.readTemperature(true);

        if (isnan(h) || isnan(t)) {
          this->error = true;

        } else {
          this->error = false;
          hi = sensor.computeHeatIndex(t, h);
        }

        display_stale = true;
      }

      runned();
    }
};

template <int num>
class RelayThread : public Thread {
  public:
    bool values[num];
    int count, pins[num];

    const unsigned char** icons;

    const char* identifiers;

    RelayThread(const int p[num], const char* ids, const unsigned char** i) : Thread(), count(num), icons(i), identifiers(ids) {
      for (int i = 0; i < num; ++i) {
        values[i] = false;
      }

      for (int i = 0; i < num; ++i) {
        pins[i] = p[i];
      }
    }

    void setup() {
      for (int i = 0; i < num; ++i) {
        pinMode(pins[i], OUTPUT);
      }

      this->run();
    }

    void run() {
      for (int i = 0; i < num; ++i) {
        digitalWrite(pins[i], values[i] ? LOW : HIGH);
      }

      runned();
    }
};

TouchSensorThread<2, float> touch(touch_pins, touch_pin_values);
DoorSensorThread door(DOORINPIN, 50, DOOROUTPIN);
DHTSensorThread dht(DHTPIN, DHTTYPE);
RelayThread<4> relay(relay_pins, relay_identifiers, relay_icons);

class LogicThread : public Thread {
  public:
    float raw_threshold, threshold;

    bool threshold_modified;

    uint32_t last_modified;

    LogicThread() : Thread(), raw_threshold(THRESHOLD_MID), threshold(THRESHOLD_MID), threshold_modified(false), last_modified(0) {}

    void run() {
      auto now = millis();

      float hi = dht.hi,
            h  = dht.h;

      auto& relays = relay.values;

      auto last_active = touch.getLastActive();

      if (touch.any_active) {
        bool new_touch = last_modified < last_active->start;

        if ((!threshold_modified || new_touch) && (now - last_active->start) < 1000) {
          //For a new touch, set the raw value to the rounded value to give a level playing field
          if (new_touch) {
            raw_threshold = threshold;
          }

          raw_threshold += last_active->value;

        } else if (last_active->time - last_active->start > 200) {
          float increment = float(now - last_modified) * 0.01;

          if (last_active->value < 0) {
            raw_threshold -= increment;

          } else {
            raw_threshold += increment;
          }
        }

        raw_threshold = constrain(raw_threshold, THRESHOLD_MIN, THRESHOLD_MAX);

        float new_threshold = floor(raw_threshold / last_active->value) * last_active->value; //This basically rounds it to the nearest unit used in the interval

        //If the difference between new and old threshold is significant, display it
        if (fabs(new_threshold - threshold) >= 0.01) {
          display_stale = true;
        }

        threshold = new_threshold;

        last_modified = now;

        threshold_modified = true;

        //Because reading from the DHT sensor can take up to a second, it can cause painful hiccups when displaying the responsive threshold,
        // so we should pause it until the screen no longer needs to be responsive
        dht.paused = true;

      } else if (threshold_modified && (now - last_active->time) > 1000) {
        threshold_modified = false;

        //Unpause the DHT sensor now that the screen is back to displaying static data
        dht.paused = false;

        display_stale = true;
      }

      float higher = threshold + HIGHER_TEMP;

      if (hi > threshold) {
        relays[TABLEFAN] = true;
        relays[DOORFAN] = true;

      } else if (hi <= (threshold - TEMP_HYSTERESIS)) {
        relays[DOORFAN] = false;
        relays[DESKFAN] = false;
        relays[TABLEFAN] = false;
      }

      if (hi > higher) {
        relays[DESKFAN] = true;

      } else if (hi <= (higher - TEMP_HYSTERESIS)) {
        relays[DESKFAN] = false;
      }

      //Don't bother turning on the fan outside the door if the door is closed
      if (!door.is_open) {
        relays[DOORFAN] = false;

      } else {
        //If the doorfan was never enabled and the temperature is within hysterisis, this will enable it if the table fan is enabled
        relays[DOORFAN] |= relays[TABLEFAN];
      }

      if (hi > threshold && h > HIGH_HUMIDITY) {
        relays[DEHUMIDIFIER] = true;

      } else if (hi <= (threshold - TEMP_HYSTERESIS) || h <= (HIGH_HUMIDITY - HUMIDITY_HYSTERESIS)) {
        relays[DEHUMIDIFIER] = false;
      }

      runned();
    }

} logic;

class DisplayThread : public Thread {
  public:
    Adafruit_SSD1306 display;
    unsigned int address;

    DisplayThread(unsigned int a) : Thread(), display(-1), address(a) {
    }

    inline void setup() {
      display.begin(SSD1306_SWITCHCAPVCC, address);
    }

    void run() {
      static char buffer[128];
      static PString out(buffer, sizeof(buffer));

      if (display_stale) {
        float &hi = dht.hi, &t = dht.t, &h = dht.h;

        auto& relays = relay.values;
        auto& identifiers = relay.identifiers;
        auto& icons = relay.icons;

        auto& temp_threshold = logic.threshold;

        out.begin(); //Reset string buffer for every refresh

        display.clearDisplay();

        display.setTextColor(1, 0);

        display.setTextSize(2); //Set text size to 2 for the header

        for (int i = 0; i < relay.count; i++) {
          if (relays[i]) {
            display.drawBitmap(i * ICON_WIDTH * 2, 0, icons[i], ICON_WIDTH, ICON_HEIGHT, 1);
            display.setCursor(i * ICON_WIDTH * 2 + ICON_WIDTH, 0);
            display.print(identifiers[i]);
          }
        }

        display.setTextSize(1); //Set it back to 1 for the main content

        if (logic.threshold_modified) {
          uint16_t x, y, w, h;
          int16_t x1, y1;

          out.print(temp_threshold);

          display.setFont(&FreeSans24pt7b); //This font is large enough that setTextSize(1) is good

          display.getTextBounds(const_cast<char*>((const char*)out), 0, 0, &x1, &y1, &w, &h);

          //Center text on screen
          x = (0 - x1) + ((display.width() - w) / 2);
          y = (OLED_HEADER_HEIGHT - y1) + ((display.height() - OLED_HEADER_HEIGHT - h) / 2);

          display.setCursor(x, y);

        } else {
          //For normal text, just set the cursor after the header
          display.setCursor(0, OLED_HEADER_HEIGHT);

          if (dht.error) {
            out.println("Failed to read from DHT sensor!");

          } else {
            out.print("Humidity:    ");
            out.print(h);
            out.print(" %\nTemp:        ");
            out.print(t);
            out.print(" F\nHeat index:  ");
            out.print(hi);
            out.print(" F\nThreshold:   ");
            out.print(temp_threshold);
            out.print(" F\n");

            out.println( door.is_open ? "Door is open" : "Door is closed");
          }
        }

        display.print(out);

        display.display();

        display.setFont(); //Reset font for next iteration

        display_stale = false;
      }

      runned();
    }

} display(OLED_ADDRESS);

void serialUpdate() {
  if (dht.error) {
    Serial.println("Failed to read from DHT sensor!");

  } else {
    Serial.print("door: ");
    Serial.print(door.is_open ? "Open\t" : "Closed\t");

    Serial.print("Humidity: ");
    Serial.print(dht.h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(dht.t);
    Serial.print(" *F\t");
    Serial.print("Heat index: ");
    Serial.print(dht.hi);
    Serial.println(" *F");
    Serial.print("Threshold: ");
    Serial.print(logic.threshold);
    Serial.println(" *F");
  }
}

Thread serial_thread(&serialUpdate, 2000);

void setup() {
  relay.setup(); //Important that this goes first so there is no chance the relays can turn on

  Serial.begin(9600);

  door.setup();
  dht.setup();
  display.setup();

  touch.setInterval(10);
  door.setInterval(10);
  dht.setInterval(2000);
  logic.setInterval(10);
  relay.setInterval(25);
  display.setInterval(1000 / 60);

  controller.add(&touch);
  controller.add(&door);
  controller.add(&dht);
  controller.add(&logic);
  controller.add(&relay);
  controller.add(&display);
  controller.add(&serial_thread);

  Serial.println("Initialized.");
}

void loop() {
  controller.run();
}
