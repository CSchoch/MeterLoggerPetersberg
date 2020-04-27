// Debug utilities
#define OFF -1
#define NONE 0
#define ERROR 1
#define WARNING 2
#define INFO 3
#define DEBUG 4
#define VERBOSE 5

#if DEBUGLEVEL >= 0
#define DEBUGPRINTNONE(...) Serial.print(__VA_ARGS__)
#define DEBUGPRINTLNNONE(...) Serial.println(__VA_ARGS__)
#define DEBUGPRINTFNONE(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUGPRINTNONE(...)
#define DEBUGPRINTLNNONE(...)
#define DEBUGPRINTFNONE(...)
#endif

#if DEBUGLEVEL >= 1
#define DEBUGPRINTERROR(...) Serial.print(__VA_ARGS__)
#define DEBUGPRINTLNERROR(...) Serial.println(__VA_ARGS__)
#define DEBUGPRINTFERROR(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUGPRINTERROR(...)
#define DEBUGPRINTLNERROR(...)
#define DEBUGPRINTFERROR(...)
#endif

#if DEBUGLEVEL >= 2
#define DEBUGPRINTWARNING(...) Serial.print(__VA_ARGS__)
#define DEBUGPRINTLNWARNING(...) Serial.println(__VA_ARGS__)
#define DEBUGPRINTFWARNING(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUGPRINTWARNING(...)
#define DEBUGPRINTLNWARNING(...)
#define DEBUGPRINTFWARNING(...)
#endif

#if DEBUGLEVEL >= 3
#define DEBUGPRINTINFO(...) Serial.print(__VA_ARGS__)
#define DEBUGPRINTLNINFO(...) Serial.println(__VA_ARGS__)
#define DEBUGPRINTFINFO(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUGPRINTINFO(...)
#define DEBUGPRINTLNINFO(...)
#define DEBUGPRINTFINFO(...)
#endif

#if DEBUGLEVEL >= 4
#define DEBUGPRINTDEBUG(...) Serial.print(__VA_ARGS__)
#define DEBUGPRINTLNDEBUG(...) Serial.println(__VA_ARGS__)
#define DEBUGPRINTFDEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUGPRINTDEBUG(...)
#define DEBUGPRINTLNDEBUG(...)
#define DEBUGPRINTFDEBUG(...)
#endif

#if DEBUGLEVEL >= 5
#define DEBUGPRINTVERBOSE(...) Serial.print(__VA_ARGS__)
#define DEBUGPRINTLNVERBOSE(...) Serial.println(__VA_ARGS__)
#define DEBUGPRINTFVERBOSE(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUGPRINTVERBOSE(...)
#define DEBUGPRINTLNVERBOSE(...)
#define DEBUGPRINTFVERBOSE(...)
#endif