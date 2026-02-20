#ifndef LAN_COMMAND_3333_H
#define LAN_COMMAND_3333_H

#include <Arduino.h>
#include <WiFi.h>

class LanCommand {
public:
  using Callback  = void (*)();

  // Hook สำหรับให้ class อื่น “จับคำสั่งบางชนิด” เช่น MON ...
  // คืนค่า true = จัดการแล้ว / false = ให้ LanCommand จัดการต่อ
  using LineHook = bool (*)(WiFiClient& client, const String& line);

  LanCommand() : server(3333) {}

  bool begin(const char* ssid, const char* pass) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(200);
      if (millis() - t0 > 15000) return false;
    }

    server.begin();
    server.setNoDelay(true);
    return true;
  }

  void onA(Callback cb) { cbA = cb; }
  void onB(Callback cb) { cbB = cb; }
  void onC(Callback cb) { cbC = cb; }
  void onReset(Callback cb) { cbReset = cb; }

  // ===== เพิ่ม: ตั้ง hook =====
  void setLineHook(LineHook hook) { lineHook = hook; }

  // ===== เพิ่ม: เช็ค/ดึง client ไปใช้ข้างนอก =====
  bool hasClient() const { return client && client.connected(); }

  WiFiClient& clientRef() { return client; }  // ใช้คู่กับ hasClient()

  void update() {
    // accept client
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      if (client) {
        client.setNoDelay(true);
        send("CONNECTED\n");
      }
    }

    if (!client || !client.connected()) return;

    while (client.available()) {
      char c = client.read();
      if (c == '\r') continue;

      if (c == '\n') {
        buffer.trim();
        handle(buffer);
        buffer = "";
      } else {
        if (buffer.length() < 100) buffer += c;
      }
    }
  }

  void send(const String& msg) {
    if (client && client.connected()) client.print(msg);
  }

private:
  WiFiServer server;
  WiFiClient client;
  String buffer;

  Callback cbA = nullptr;
  Callback cbB = nullptr;
  Callback cbC = nullptr;
  Callback cbReset = nullptr;

  // ===== เพิ่ม: hook pointer =====
  LineHook lineHook = nullptr;

  void handle(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // ===== เพิ่ม: ให้ hook จับก่อน (เช่น MON ...) =====
    // หมายเหตุ: ส่ง cmd ตัวเดิม (ยังไม่ toLowerCase) เพื่อให้ hook เลือก parse เองได้
    if (lineHook) {
      if (lineHook(client, cmd)) {
        return; // ถูกจัดการแล้ว
      }
    }

    // คำสั่งเดิมของคุณ
    cmd.toLowerCase();

    if (cmd == "reset") {
      if (cbReset) cbReset();
      send("OK reset\n");
      return;
    }

    if (cmd == "a") {
      if (cbA) cbA();
      send("OK A\n");
      return;
    }

    if (cmd == "b") {
      if (cbB) cbB();
      send("OK B\n");
      return;
    }

    if (cmd == "c") {
      if (cbC) cbC();
      send("OK C\n");
      return;
    }

    send("ERR\n");
  }
};

#endif
