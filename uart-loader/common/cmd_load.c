/*
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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

/*
 * Serial up- and download support
 */
#include <common.h>
#include <asm/arch/mux.h>

#define putc serial_putc
#define tstc serial_tstc

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}
static inline void udelay(unsigned long us)
{
	delay(us * 200); /* approximate */
}

static gpio_t *blocks[]={ // GPIO1 .. 6
	OMAP34XX_GPIO1_BASE,
	OMAP34XX_GPIO2_BASE,
	OMAP34XX_GPIO3_BASE,
	OMAP34XX_GPIO4_BASE,
	OMAP34XX_GPIO5_BASE,
	OMAP34XX_GPIO6_BASE
};

static inline int gpio_get(int n)
{
	int bit=n % 32;
	gpio_t *base=blocks[n/32];
	return (base->datain >> bit)&1;
}

static inline int gpio_is_input(int n)
{ // geht nicht richtig
	int bit=n % 32;
	gpio_t *base=blocks[n/32];
	return (base->oe >> bit)&1;
}

#ifdef CFG_CMD_FAT
extern void * memcpy(void * dest,const void *src,size_t count);
#else
void * memcpy(void * dest,const void *src,size_t count)
{
	char *tmp = (char *) dest, *s = (char *) src;

	while (count--)
		*tmp++ = *s++;

	return dest;
}
#endif

/* -------------------------------------------------------------------- */

#define XON_CHAR        17
#define XOFF_CHAR       19
#define START_CHAR      0x01
#define ETX_CHAR	0x03
#define END_CHAR        0x0D
#define SPACE           0x20
#define K_ESCAPE        0x23
#define SEND_TYPE       'S'
#define DATA_TYPE       'D'
#define ACK_TYPE        'Y'
#define NACK_TYPE       'N'
#define BREAK_TYPE      'B'
#define tochar(x) ((char) (((x) + SPACE) & 0xff))
#define untochar(x) ((int) (((x) - SPACE) & 0xff))

extern int os_data_count;
extern int os_data_header[8];

static void set_kerm_bin_mode(unsigned long *);
static int k_recv(void);
static ulong load_serial_bin (ulong offset);


char his_eol;        /* character he needs at end of packet */
int  his_pad_count;  /* number of pad chars he needs */
char his_pad_char;   /* pad chars he needs */
char his_quote;      /* quote chars he'll use */

int strcmp(char *s1, char *s2)
{
	while(*s1 == *s2) {
		if(*s1 == 0)
			return 0;	// same
		s1++;
		s2++;		
	}
	return *s1 > *s2?1:-1;
}

long xtol(char *s)
{
	long val=0;
	while(1)
		{
		if(*s >= '0' && *s <= '9')
			val=16*val+(*s++-'0');
		else if(*s >= 'A' && *s <= 'F')
			val=16*val+(*s++-'A'+10);
		else if(*s >= 'a' && *s <= 'f')
			val=16*val+(*s++-'a'+10);
		else
			break;
		}
	return val;
}

static char *addr=(char *) CFG_LOADADDR;
static char line[100];
static char *argv[10];

#define __raw_readl(a)    (*(volatile unsigned int *)(a))
#define __raw_writel(v,a) (*(volatile unsigned int *)(a) = (v))
#define __raw_readw(a)    (*(volatile unsigned short *)(a))
#define __raw_writew(v,a) (*(volatile unsigned short *)(a) = (v))

#define 	MUX_VAL(OFFSET,VALUE)\
__raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)

void testfn(void)
{ // code can be copied to SDRAM (assuming that it is position-independent!)
	int i;
	for(i=0; i<10; i++) {
		if(i & 1)
			MUX_VAL(CP(GPMC_nCS6),      (IEN | PTD | EN  | M4)) /*GPT_PWM11/GPIO57*/
		else
			MUX_VAL(CP(GPMC_nCS6),      (IEN | PTU | EN  | M4)) /*GPT_PWM11/GPIO57*/
		udelay(500*1000);
	}
}

int do_comand_line(void)
{ // add testing code here
	printf ("Welcome to uart-loader\n");
	while(1) {
		char *c;
		int argc=0;
		int pos=0;
		printf ("$ ");
		while(1) {
			line[pos]=getc();
			switch(line[pos]) {
				case '\b':
				case 0x7f:
					if(pos > 0) {
						putc('\b');
						pos--;
					}
					continue;
				case 0:
				case '\n':
					continue;	// ignore
				case '\r':
					break;	// command
				default:
					if(pos < sizeof(line)/sizeof(line[0])) {
						putc(line[pos]);
						pos++;
					}
					continue;
			}
			line[pos]=0;
			printf("\n");
			break;
		}
		c=line;
		while(argc < sizeof(argv)/sizeof(argv[0])) {
			while (*c == ' ' || *c == '\t')
				c++;
			if(*c == 0)
				break;
			argv[argc]=c;
			while(*c != 0 && *c != '\t' && *c != ' ')
				c++;
			argc++;
			if(*c == 0)
				break;
			*c=0;	// substitute
			c++;
		}
		if(argc == 0)
			continue;	// empty line
//		printf("argc=%d argv[0]=%s\n", argc, argv[0]);
		if(strcmp(argv[0], "c") == 0) { // c - copy test function to SDRAM and execute
			void (*fn)(void) = &testfn;
			size_t cnt = 0x1000;
			addr=(char *) CFG_LOADADDR;
			printf("%08x (%d) --> %08x\n", fn, cnt, addr);
			memcpy(addr, fn, cnt);
			(*fn)();
		}
		else if(strcmp(argv[0], "x") == 0) { // x [addr] - execute code
			void (*fn)(void) = addr;
			if(argc > 1)
				fn = (void *) xtol(argv[1]);
			(*fn)();
		}
		else if(strcmp(argv[0], "l") == 0) { // l - loop
			int d=1;
			while(1)
				printf ("Welcome to uart-monitor (%d)\n", d++), udelay(500*1000);
		}
		else if(strcmp(argv[0], "ram") == 0) {
			int val=0x55;
			if(argc == 2)
				val=xtol(argv[1]);
			for(addr=(char *) CFG_LOADADDR; addr < 256+(char *) CFG_LOADADDR; addr++) {
				*addr=val++;
			}
			addr=(char *) CFG_LOADADDR;
		}
		else if(strcmp(argv[0], "a") == 0) { // a [addr]
			if(argc > 1)
				addr=(char *) xtol(argv[1]);
			printf("%08x\n", addr);
		}
		else if(strcmp(argv[0], "r") == 0) { // r [size]
			int i;
			int n=1;
			if(argc > 1)
				n=xtol(argv[1]);
			printf("%08x:", addr);
			for(i=0; i<n; i++) {
				printf(" %02x", *addr++);
				if(i % 16 == 15 && i != n-1)
					printf("\n%08x:", addr);
			}
			printf("\n");
		}
		else if(strcmp(argv[0], "rl") == 0) { // rl [size]
			int i;
			int n=1;
			if(argc > 1)
				n=xtol(argv[1]);
			printf("%08x:", addr);
			for(i=0; i<n; i++) {
				printf(" %08x", *(volatile unsigned int *) addr);
				addr+=4;
				if(i % 4 == 3 && i != n-1)
					printf("\n%08x:", addr);
			}
			printf("\n");
		}
		else if(strcmp(argv[0], "f") == 0) { // f [value [size]]
			int i;
			int n=1;
			int val=0x55;
			if(argc > 1)
				val=xtol(argv[1]);
			if(argc > 2)
				n=xtol(argv[2]);
			for(i=0; i<n; i++)
				addr[i]=val;
		}
#if 0
		else if(strcmp(argv[0], "clk") == 0) { // clk
			extern u32 osc_clk;
			printf("clk=%d\n", osc_clk);
		}
#endif
		else if(strcmp(argv[0], "bl") == 0) { // blink backlight
			int i;
			for(i=0; i<10; i++) {
				if(i & 1)
					MUX_VAL(CP(GPMC_nCS6),      (IEN | PTD | EN  | M4)) /*GPT_PWM11/GPIO57*/
				else
					MUX_VAL(CP(GPMC_nCS6),      (IEN | PTU | EN  | M4)) /*GPT_PWM11/GPIO57*/
				udelay(500*1000);
			}
		}
		else if(strcmp(argv[0], "g") == 0) { // g - read GPIO
			unsigned int i;	// gcc compilation has problems with signed division and modulo!
			int n=32*sizeof(blocks)/sizeof(blocks[0]);
			printf("GPIO1 data: %08x\n", ((gpio_t *) OMAP34XX_GPIO1_BASE) -> datain);
			printf("%03d:", 0);
			for(i=0; i<n; i++) {
				printf(" %c%d", (gpio_is_input(i)?' ':'o'), gpio_get(i));
				if(i % 10 == 9 && i != n-1)
					printf("\n%03d:", i);
			}
			printf("\n");
			
		}
		else if(strcmp(argv[0], "m") == 0) { // m - read pinmux
			int cols=0;
			addr=0x48002030;
			printf("%08x", addr);					
			while(addr <= 0x480025F8) {
				unsigned mux=*(unsigned *) addr;
				int i;
				for(i=1; i <= 2; i++) {
					printf(" %c%d%c", (mux&8)?((mux&0x10?'U':'D')):' ', (mux&7), (mux&0x100)?'I':'O');
					mux >>= 16;
				}
				if(addr == 0x48002264) {
					addr=0x480025DC;
					printf("\n%08x", addr);
					cols=0;
				}
				else {
					addr+=4;
					if(++cols == 8)
						printf("\n%08x", addr), cols=0;
				}
			}
			printf("\n");
		}
		else
			printf("uart-loader unknown command: %s\n", argv[0]);
	}
	return 0;
}

int do_load_serial_bin (ulong offset, int baudrate)
{
	ulong addr;
	int rcode = 0;

	printf ("## Ready for binary (kermit) download "
		"to 0x%08lX at %d bps...\n",
		offset,
		baudrate);
	addr = load_serial_bin (offset);
	
	udelay(3*1000*1000);
	
	if (addr == ~0) {
		printf ("## Binary (kermit) download aborted\n");
		rcode = 1;
	} else {
		printf ("## Start Addr      = 0x%08lX\n", addr);
#if 0
		udelay(3*1000*1000);
			{
			int i;
			int n=256;
			char *a=(char *) addr;
			printf("%08x:", a);
			for(i=0; i<n; i++) {
				printf(" %02x", *a++);
				if(i % 16 == 15 && i != n-1)
					printf("\n%08x:", a+1);
			}
			printf("\n");
		}
#endif	
	}
	return rcode;
}


static ulong load_serial_bin (ulong offset)
{
	int size, i;

	set_kerm_bin_mode ((ulong *) offset);
	size = k_recv ();

	/*
	 * Gather any trailing characters (for instance, the ^D which
	 * is sent by 'cu' after sending a file), and give the
	 * box some time (100 * 1 ms)
	 */
	for (i=0; i<1000; ++i) {
		if (tstc()) {
			(void) getc();
		}
		udelay(1000);
	}

	udelay(5*1000*1000);
	printf("## Total Size      = 0x%08x = %d Bytes\n", size, size);

	return offset;
}

void send_pad (void)
{
	int count = his_pad_count;

	while (count-- > 0)
		putc (his_pad_char);
}

/* converts escaped kermit char to binary char */
char ktrans (char in)
{
	if ((in & 0x60) == 0x40) {
		return (char) (in & ~0x40);
	} else if ((in & 0x7f) == 0x3f) {
		return (char) (in | 0x40);
	} else
		return in;
}

int chk1 (char *buffer)
{
	int total = 0;

	while (*buffer) {
		total += *buffer++;
	}
	return (int) ((total + ((total >> 6) & 0x03)) & 0x3f);
}

void s1_sendpacket (char *packet)
{
	send_pad ();
	while (*packet) {
		putc (*packet++);
	}
}

static char a_b[24];
void send_ack (int n)
{
	a_b[0] = START_CHAR;
	a_b[1] = tochar (3);
	a_b[2] = tochar (n);
	a_b[3] = ACK_TYPE;
	a_b[4] = '\0';
	a_b[4] = tochar (chk1 (&a_b[1]));
	a_b[5] = his_eol;
	a_b[6] = '\0';
	s1_sendpacket (a_b);
}

void send_nack (int n)
{
	a_b[0] = START_CHAR;
	a_b[1] = tochar (3);
	a_b[2] = tochar (n);
	a_b[3] = NACK_TYPE;
	a_b[4] = '\0';
	a_b[4] = tochar (chk1 (&a_b[1]));
	a_b[5] = his_eol;
	a_b[6] = '\0';
	s1_sendpacket (a_b);
}


/* os_data_* takes an OS Open image and puts it into memory, and
   puts the boot header in an array named os_data_header

   if image is binary, no header is stored in os_data_header.
*/
void (*os_data_init) (void);
void (*os_data_char) (char new_char);
static int os_data_state, os_data_state_saved;
int os_data_count;
static int os_data_count_saved;
static char *os_data_addr, *os_data_addr_saved;
static char *bin_start_address;
int os_data_header[8];
static void bin_data_init (void)
{
	os_data_state = 0;
	os_data_count = 0;
	os_data_addr = bin_start_address;
}
static void os_data_save (void)
{
	os_data_state_saved = os_data_state;
	os_data_count_saved = os_data_count;
	os_data_addr_saved = os_data_addr;
}
static void os_data_restore (void)
{
	os_data_state = os_data_state_saved;
	os_data_count = os_data_count_saved;
	os_data_addr = os_data_addr_saved;
}
static void bin_data_char (char new_char)
{
	switch (os_data_state) {
	case 0:					/* data */
		*os_data_addr++ = new_char;
		--os_data_count;
		break;
	}
}
static void set_kerm_bin_mode (unsigned long *addr)
{
	bin_start_address = (char *) addr;
	os_data_init = bin_data_init;
	os_data_char = bin_data_char;
}


/* k_data_* simply handles the kermit escape translations */
static int k_data_escape, k_data_escape_saved;
void k_data_init (void)
{
	k_data_escape = 0;
	os_data_init ();
}
void k_data_save (void)
{
	k_data_escape_saved = k_data_escape;
	os_data_save ();
}
void k_data_restore (void)
{
	k_data_escape = k_data_escape_saved;
	os_data_restore ();
}
void k_data_char (char new_char)
{
	if (k_data_escape) {
		/* last char was escape - translate this character */
		os_data_char (ktrans (new_char));
		k_data_escape = 0;
	} else {
		if (new_char == his_quote) {
			/* this char is escape - remember */
			k_data_escape = 1;
		} else {
			/* otherwise send this char as-is */
			os_data_char (new_char);
		}
	}
}

#define SEND_DATA_SIZE  20
char send_parms[SEND_DATA_SIZE];
char *send_ptr;

/* handle_send_packet interprits the protocol info and builds and
   sends an appropriate ack for what we can do */
void handle_send_packet (int n)
{
	int length = 3;
	int bytes;

	/* initialize some protocol parameters */
	his_eol = END_CHAR;		/* default end of line character */
	his_pad_count = 0;
	his_pad_char = '\0';
	his_quote = K_ESCAPE;

	/* ignore last character if it filled the buffer */
	if (send_ptr == &send_parms[SEND_DATA_SIZE - 1])
		--send_ptr;
	bytes = send_ptr - send_parms;	/* how many bytes we'll process */
	do {
		if (bytes-- <= 0)
			break;
		/* handle MAXL - max length */
		/* ignore what he says - most I'll take (here) is 94 */
		a_b[++length] = tochar (94);
		if (bytes-- <= 0)
			break;
		/* handle TIME - time you should wait for my packets */
		/* ignore what he says - don't wait for my ack longer than 1 second */
		a_b[++length] = tochar (1);
		if (bytes-- <= 0)
			break;
		/* handle NPAD - number of pad chars I need */
		/* remember what he says - I need none */
		his_pad_count = untochar (send_parms[2]);
		a_b[++length] = tochar (0);
		if (bytes-- <= 0)
			break;
		/* handle PADC - pad chars I need */
		/* remember what he says - I need none */
		his_pad_char = ktrans (send_parms[3]);
		a_b[++length] = 0x40;	/* He should ignore this */
		if (bytes-- <= 0)
			break;
		/* handle EOL - end of line he needs */
		/* remember what he says - I need CR */
		his_eol = untochar (send_parms[4]);
		a_b[++length] = tochar (END_CHAR);
		if (bytes-- <= 0)
			break;
		/* handle QCTL - quote control char he'll use */
		/* remember what he says - I'll use '#' */
		his_quote = send_parms[5];
		a_b[++length] = '#';
		if (bytes-- <= 0)
			break;
		/* handle QBIN - 8-th bit prefixing */
		/* ignore what he says - I refuse */
		a_b[++length] = 'N';
		if (bytes-- <= 0)
			break;
		/* handle CHKT - the clock check type */
		/* ignore what he says - I do type 1 (for now) */
		a_b[++length] = '1';
		if (bytes-- <= 0)
			break;
		/* handle REPT - the repeat prefix */
		/* ignore what he says - I refuse (for now) */
		a_b[++length] = 'N';
		if (bytes-- <= 0)
			break;
		/* handle CAPAS - the capabilities mask */
		/* ignore what he says - I only do long packets - I don't do windows */
		a_b[++length] = tochar (2);	/* only long packets */
		a_b[++length] = tochar (0);	/* no windows */
		a_b[++length] = tochar (94);	/* large packet msb */
		a_b[++length] = tochar (94);	/* large packet lsb */
	} while (0);

	a_b[0] = START_CHAR;
	a_b[1] = tochar (length);
	a_b[2] = tochar (n);
	a_b[3] = ACK_TYPE;
	a_b[++length] = '\0';
	a_b[length] = tochar (chk1 (&a_b[1]));
	a_b[++length] = his_eol;
	a_b[++length] = '\0';
	s1_sendpacket (a_b);
}

/* k_recv receives a OS Open image file over kermit line */
static int k_recv (void)
{
	char new_char;
	char k_state, k_state_saved;
	int sum;
	int done;
	int length;
	int n, last_n;
	int z = 0;
	int len_lo, len_hi;

	/* initialize some protocol parameters */
	his_eol = END_CHAR;		/* default end of line character */
	his_pad_count = 0;
	his_pad_char = '\0';
	his_quote = K_ESCAPE;

	/* initialize the k_recv and k_data state machine */
	done = 0;
	k_state = 0;
	k_data_init ();
	k_state_saved = k_state;
	k_data_save ();
	n = 0;				/* just to get rid of a warning */
	last_n = -1;

	/* expect this "type" sequence (but don't check):
	   S: send initiate
	   F: file header
	   D: data (multiple)
	   Z: end of file
	   B: break transmission
	 */

	/* enter main loop */
	while (!done) {
		/* set the send packet pointer to begining of send packet parms */
		send_ptr = send_parms;

		/* With each packet, start summing the bytes starting with the length.
		   Save the current sequence number.
		   Note the type of the packet.
		   If a character less than SPACE (0x20) is received - error.
		 */

#if 0
		/* OLD CODE, Prior to checking sequence numbers */
		/* first have all state machines save current states */
		k_state_saved = k_state;
		k_data_save ();
#endif

		/* get a packet */
		/* wait for the starting character or ^C */
		for (;;) {
			switch (getc ()) {
			case START_CHAR:	/* start packet */
				goto START;
			case ETX_CHAR:		/* ^C waiting for packet */
				return (0);
			default:
				;
			}
		}
START:
		/* get length of packet */
		sum = 0;
		new_char = getc ();
		if ((new_char & 0xE0) == 0)
			goto packet_error;
		sum += new_char & 0xff;
		length = untochar (new_char);
		/* get sequence number */
		new_char = getc ();
		if ((new_char & 0xE0) == 0)
			goto packet_error;
		sum += new_char & 0xff;
		n = untochar (new_char);
		--length;

		/* NEW CODE - check sequence numbers for retried packets */
		/* Note - this new code assumes that the sequence number is correctly
		 * received.  Handling an invalid sequence number adds another layer
		 * of complexity that may not be needed - yet!  At this time, I'm hoping
		 * that I don't need to buffer the incoming data packets and can write
		 * the data into memory in real time.
		 */
		if (n == last_n) {
			/* same sequence number, restore the previous state */
			k_state = k_state_saved;
			k_data_restore ();
		} else {
			/* new sequence number, checkpoint the download */
			last_n = n;
			k_state_saved = k_state;
			k_data_save ();
		}
		/* END NEW CODE */

		/* get packet type */
		new_char = getc ();
		if ((new_char & 0xE0) == 0)
			goto packet_error;
		sum += new_char & 0xff;
		k_state = new_char;
		--length;
		/* check for extended length */
		if (length == -2) {
			/* (length byte was 0, decremented twice) */
			/* get the two length bytes */
			new_char = getc ();
			if ((new_char & 0xE0) == 0)
				goto packet_error;
			sum += new_char & 0xff;
			len_hi = untochar (new_char);
			new_char = getc ();
			if ((new_char & 0xE0) == 0)
				goto packet_error;
			sum += new_char & 0xff;
			len_lo = untochar (new_char);
			length = len_hi * 95 + len_lo;
			/* check header checksum */
			new_char = getc ();
			if ((new_char & 0xE0) == 0)
				goto packet_error;
			if (new_char != tochar ((sum + ((sum >> 6) & 0x03)) & 0x3f))
				goto packet_error;
			sum += new_char & 0xff;
/* --length; */ /* new length includes only data and block check to come */
		}
		/* bring in rest of packet */
		while (length > 1) {
			new_char = getc ();
			if ((new_char & 0xE0) == 0)
				goto packet_error;
			sum += new_char & 0xff;
			--length;
			if (k_state == DATA_TYPE) {
				/* pass on the data if this is a data packet */
				k_data_char (new_char);
			} else if (k_state == SEND_TYPE) {
				/* save send pack in buffer as is */
				*send_ptr++ = new_char;
				/* if too much data, back off the pointer */
				if (send_ptr >= &send_parms[SEND_DATA_SIZE])
					--send_ptr;
			}
		}
		/* get and validate checksum character */
		new_char = getc ();
		if ((new_char & 0xE0) == 0)
			goto packet_error;
		if (new_char != tochar ((sum + ((sum >> 6) & 0x03)) & 0x3f))
			goto packet_error;
		/* get END_CHAR */
		new_char = getc ();
		if (new_char != END_CHAR) {
		  packet_error:
			/* restore state machines */
			k_state = k_state_saved;
			k_data_restore ();
			/* send a negative acknowledge packet in */
			send_nack (n);
		} else if (k_state == SEND_TYPE) {
			/* crack the protocol parms, build an appropriate ack packet */
			handle_send_packet (n);
		} else {
			/* send simple acknowledge packet in */
			send_ack (n);
			/* quit if end of transmission */
			if (k_state == BREAK_TYPE)
				done = 1;
		}
		++z;
	}
	return ((ulong) os_data_addr - (ulong) bin_start_address);
}
