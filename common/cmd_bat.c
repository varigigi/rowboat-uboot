/*
 * Copyright (C) 2010 Texas Instruments
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>

#ifdef CONFIG_CMD_BAT
#include <twl6030.h>

int do_vbat(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char *cmd;
	if (argc == 2) {
		if (strncmp(argv[1], "init", 5) == 0)
			twl6030_init_battery_charging();
		else if (strncmp(argv[1], "startcharge", 12) == 0)
			twl6030_start_usb_charging();
		else if (strncmp(argv[1], "stopcharge", 11) == 0)
			twl6030_stop_usb_charging();
		else if (strncmp(argv[1], "status", 7) == 0) {
			twl6030_get_battery_voltage();
			twl6030_get_battery_current();
		} else {
			goto bat_cmd_usage;
		}
	} else {
		goto bat_cmd_usage;
	}
	return 0;

bat_cmd_usage:
	printf("Usage:\n%s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(
	bat, 2, 1, do_vbat,
	"bat     - battery charging, voltage measurement\n",
	"status - display battery voltage and current\n"
	"bat startcharge - start charging via USB\n"
	"bat stopcharge - stop charging\n"
);
#endif /* CONFIG_BAT_CMD */
