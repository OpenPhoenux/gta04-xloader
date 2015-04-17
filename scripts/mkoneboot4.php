<?php
/*
 * helper script to duplicate all blocks for GTA04 OneNAND chip
 * with 4k block size
 * OMAP3 TRM tells that in such a case the BootROM reads
 * sectors 0,1,2,3 and skips 4,5,6,7
 *
 * use this after running signGP
 *
 * usage: php scripts/mkoneboot4.php <x-load.bin.ift >x-load.bin.flash
 */

while($block=fread(STDIN, 2048))
	{
	fwrite(STDOUT, $block);
	fwrite(STDOUT, $block);
	}
exit;
?>