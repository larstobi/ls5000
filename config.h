#define HAVE_LIBUSB 1

extern const char *sanei_config_skip_whitespace(const char *name);
extern const char *sanei_config_get_string(const char *name, char **val);

/* debug stuff */

#define __stringify(x)	#x
#define stringify(x)	__stringify(x)

extern void debug(char *name, int lvl, char *msg, ...);
extern void debug_init(char *name);
#define DBG(lvl, msg, args...) debug(stringify(BACKEND_NAME), lvl, msg,##args)

#define DBG_INIT() debug_init(stringify(BACKEND_NAME))
