#include "saved_songs.h"
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "secrets.h"

namespace SavedSongs {

static Song songs[MAX];
static int n = 0;

static void store() {
    Preferences prefs;
    prefs.begin("songs", false);
    prefs.putInt("n", n);
    if (n) prefs.putBytes("list", songs, sizeof(Song) * n);
    prefs.end();
}

void begin() {
    Preferences prefs;
    prefs.begin("songs", true);
    n = constrain((int)prefs.getInt("n", 0), 0, MAX);
    if (n) prefs.getBytes("list", songs, sizeof(Song) * n);
    prefs.end();
}

int count() { return n; }

const Song &get(int i) { return songs[i]; }

bool contains(const String &title, const String &artist) {
    for (int i = 0; i < n; i++)
        if (title == songs[i].title && artist == songs[i].artist) return true;
    return false;
}

bool add(const String &title, const String &artist) {
    if (title.isEmpty() || contains(title, artist)) return false;
    if (n == MAX) n--; // drop the oldest
    memmove(songs + 1, songs, sizeof(Song) * n);
    memset(&songs[0], 0, sizeof(Song));
    strlcpy(songs[0].title, title.c_str(), sizeof(songs[0].title));
    strlcpy(songs[0].artist, artist.c_str(), sizeof(songs[0].artist));
    n++;
    store();
    return true;
}

void remove(int i) {
    if (i < 0 || i >= n) return;
    memmove(songs + i, songs + i + 1, sizeof(Song) * (n - i - 1));
    n--;
    store();
}

static String urlEncode(const String &s) {
    static const char *HEX_DIGITS = "0123456789ABCDEF";
    String out;
    for (size_t i = 0; i < s.length(); i++) {
        uint8_t c = s[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') out += (char)c;
        else {
            out += '%';
            out += HEX_DIGITS[c >> 4];
            out += HEX_DIGITS[c & 15];
        }
    }
    return out;
}

String play(const Song &s) {
    if (strlen(MACRODROID_WEBHOOK) == 0) return "Webhook not set up";
    if (WiFi.status() != WL_CONNECTED) return "No Wi-Fi";
    String query = String(s.title) + (s.artist[0] ? String(" ") + s.artist : "");
    String url = String(MACRODROID_WEBHOOK) + "?query=" + urlEncode(query);

    WiFiClientSecure tls;
    tls.setInsecure(); // no CA pinning; fine for a home gadget
    HTTPClient http;
    http.setTimeout(6000);
    http.begin(tls, url);
    int code = http.GET();
    http.end();
    Serial.printf("play \"%s\" -> HTTP %d\n", query.c_str(), code);
    if (code == 200) return "";
    return code > 0 ? "Phone said " + String(code) : "Couldn't reach MacroDroid";
}

} // namespace SavedSongs
