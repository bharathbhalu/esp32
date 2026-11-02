// Saved songs (kept in NVS) and replaying one in YouTube Music on the phone via
// a MacroDroid webhook. The macro turns the query into Android's standard
// "play from search" intent for YouTube Music.
#pragma once
#include <Arduino.h>

namespace SavedSongs {

static const int MAX = 30;

struct Song {
    char title[64];
    char artist[40];
};

void begin();
int count();
const Song &get(int i); // newest first
// Returns false if it's already saved or the list is full
bool add(const String &title, const String &artist);
void remove(int i);
bool contains(const String &title, const String &artist);

// Asks the phone to play the song; returns an error message, or "" on success
String play(const Song &s);

} // namespace SavedSongs
