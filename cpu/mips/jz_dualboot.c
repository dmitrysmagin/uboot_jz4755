/*
 * JzRISC Recovry Support
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <config.h>
#include <asm/mipsregs.h>
	 
#include <common.h>
#include <command.h>
#if defined(CONFIG_JZ4760)
#include <asm/jz4760.h>
#elif defined(CONFIG_JZ4760B)
#include <asm/jz4760b.h>
#elif defined(CONFIG_JZ4750D)
#include <asm/jz4750d.h>
#elif defined(CONFIG_JZ4750)
#include <asm/jz4750.h>
#endif
#include <linux/stddef.h>
#include <malloc.h>

#ifdef CFG_JZ_LINUX_DUALBOOT
unsigned int is_jz_dualboot;
extern unsigned char default_environment[];
extern unsigned char minios_environment[];
extern unsigned char linux_environment[];

#define BOOT_NORMAL   			0
#define BOOT_DUALBOOT_KEY   		4	
#define BOOT_DEFAULT			BOOT_NORMAL


static inline int is_dualboot_keys_pressed(void)
{
	int key;

	__gpio_set_pin(UBOOT_SEL_DUALBOOT_KEY);

	__gpio_as_input(UBOOT_SEL_DUALBOOT_KEY);

	key = __gpio_get_pin(UBOOT_SEL_DUALBOOT_KEY);

	return !(key);
}


int dualboot_mode_check(void)
{
	//check select press, for boot linux
	if (is_dualboot_keys_pressed()){
                return BOOT_DUALBOOT_KEY;
        }

	return BOOT_NORMAL;
}
void jz_dualboot_handle(void)
{
	unsigned int signature = 0;
	
	is_jz_dualboot=dualboot_mode_check();
	puts ("\n");
	
	if (is_jz_dualboot == BOOT_DUALBOOT_KEY){ //save env here 
		puts ("Booting Dingux 320e ...\n");
	}else
		puts ("NORMAL Booting ...\n");
	puts ("\n");

	memset(default_environment,0,CONFIG_DEFAULT_ENV_SIZE);
	if(is_jz_dualboot != BOOT_NORMAL)
		memcpy(default_environment,linux_environment,CONFIG_DEFAULT_ENV_SIZE);
	else
		memcpy(default_environment,minios_environment,CONFIG_DEFAULT_ENV_SIZE);
}

#endif  /* CFG_JZ_LINUX_DUALBOOT */

