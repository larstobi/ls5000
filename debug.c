#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "config.h"

static int ls5000_lvl = 0;

void debug(char *name, int lvl, char *msg, ...)
{
	if (strcmp(name, "ls5000"))
		return;
	if (lvl > ls5000_lvl)
		return;
	va_list args;
	va_start(args, msg);
	vfprintf(stderr, msg, args);
	va_end(args);
}

void debug_init(char *name)
{
	char *lvl;
	if (strcmp(name, "ls5000"))
		return;

	lvl = getenv("SANE_DEBUG_LS5000");
	if (!lvl)
		return;

	ls5000_lvl = atoi(lvl);
}
