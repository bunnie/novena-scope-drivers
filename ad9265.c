#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "ad9265.h"

#define AD9265_I2C_ADR  0x1E  // same as FPGA address

//#define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif

#ifdef DEBUG  // discipline to only use dump in debug mode
void dump(void *buf, int count) {
  int i = 0;
  for( i = 0; i < count; i++ ) {
    if( (i%8) == 0 ) 
      printf( "\n" );
    printf( "%02x ", ((char *) buf)[i] );
  }
}
#endif

int ad9265_write_byte( unsigned char address, unsigned char data ) {
  int i2cfd;
  char i2cbuf[4]; 
  int slave_address = AD9265_I2C_ADR;
  unsigned char c, d;

  // now do a read of a byte
  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return -1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    return -1;
  }

  i2cbuf[0] = FPGA_ADC_WDATA; i2cbuf[1] = (unsigned char) data;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }

  i2cbuf[0] = FPGA_ADC_ADDR_LSB; i2cbuf[1] = (unsigned char) (address & 0xFF) ;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  c = (unsigned char)  (((address >> 8) & 0x1F) );

  i2cbuf[0] = FPGA_ADC_ADDR_COMIT; i2cbuf[1] = c;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  c |= FPGA_ADC_COMIT_MSK;

  i2cbuf[0] = FPGA_ADC_ADDR_COMIT; i2cbuf[1] = c;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  
  do {
    d = i2c_smbus_read_byte_data(i2cfd, FPGA_ADC_STAT);
  } while( d & 1 );

  // commit is not self-clearing
  c &= ~FPGA_ADC_COMIT_MSK;
  i2cbuf[0] = FPGA_ADC_ADDR_COMIT; i2cbuf[1] = c;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  
  close( i2cfd );

  return 0;
}

int ad9265_read_byte( int address, unsigned char *data ) {
  int i2cfd;
  int slave_address = AD9265_I2C_ADR;
  unsigned char c, d;
  char i2cbuf[256]; // meh too big but meh


  // now do a read of a byte
  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return -1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    return -1;
  }

#if 0
  // run a quick test to make sure the interface is working
  printf( "interface test: (should see 0xaa the 0x55)\n" );
  i2cbuf[0] = 0x0; i2cbuf[1] = 0xAA;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }

  i2c_read = i2c_smbus_read_byte_data(i2cfd, 0x40);
  printf( "read back %02x\n", i2c_read & 0xFF );

  i2cbuf[0] = 0x0; i2cbuf[1] = 0x55;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
#endif

  i2cbuf[0] = FPGA_ADC_ADDR_LSB; i2cbuf[1] = (unsigned char) (address & 0xFF) ;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  c = (unsigned char)  (((address >> 8) & 0x1F) | FPGA_ADC_RD_WR_MSK);

  i2cbuf[0] = FPGA_ADC_ADDR_COMIT; i2cbuf[1] = c;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  c |= FPGA_ADC_COMIT_MSK;

  i2cbuf[0] = FPGA_ADC_ADDR_COMIT; i2cbuf[1] = c;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  
  do {
    d = i2c_smbus_read_byte_data(i2cfd, FPGA_ADC_STAT);
  } while( d & 1 );

  *data = i2c_smbus_read_byte_data(i2cfd, FPGA_ADC_RBK);


  // commit is not self-clearing
  c &= ~FPGA_ADC_COMIT_MSK;
  i2cbuf[0] = FPGA_ADC_ADDR_COMIT; i2cbuf[1] = c;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }
  
  close( i2cfd );

  return 0;
}


void default_ad9265() {
  ad9265_write_byte(AD9265_POWERMODE, 0x80); // don't power down
  ad9265_write_byte(AD9265_GLOBALCLK, 0x01); // turn on duty cycle stabilizer
  ad9265_write_byte(AD9265_CLOCKDIV, 0x00); // don't divide the clock
  ad9265_write_byte(AD9265_TRANSFER, 0x01 ); // commit values
  ad9265_write_byte(AD9265_OUTMODE, 0x40 ); // output as LVDS, ANSI levels, don't invert output, offset binary
  ad9265_write_byte(AD9265_TRANSFER, 0x01 ); // commit values
  ad9265_write_byte(AD9265_CLOCKPHASE, 0x00 ); // no delay on clock phase
  ad9265_write_byte(AD9265_DCODELAY, 0x00 ); // no delay on DCO output
  ad9265_write_byte(AD9265_VREF, 0xc0 ); // VREF is 2.0Vp-p

  ad9265_write_byte(AD9265_TESTMODE, 0x00 ); // no test mode

  ad9265_write_byte(AD9265_TRANSFER, 0x01 ); // commit values
}

void testpattern_ad9265() {
  ad9265_write_byte(AD9265_POWERMODE, 0x80); // don't power down
  ad9265_write_byte(AD9265_GLOBALCLK, 0x01); // turn on duty cycle stabilizer
  ad9265_write_byte(AD9265_CLOCKDIV, 0x00); // don't divide the clock
  ad9265_write_byte(AD9265_OUTMODE, 0x40 ); // output as LVDS, ANSI levels, don't invert output, offset binary
  ad9265_write_byte(AD9265_TRANSFER, 0x01 ); // commit values
  ad9265_write_byte(AD9265_CLOCKPHASE, 0x00 ); // no delay on clock phase
  ad9265_write_byte(AD9265_DCODELAY, 0x00 ); // no delay on DCO output
  ad9265_write_byte(AD9265_VREF, 0xc0 ); // VREF is 2.0Vp-p

  ad9265_write_byte(AD9265_TESTMODE, 0x06 ); // PN 9 sequence

  ad9265_write_byte(AD9265_TRANSFER, 0x01 ); // commit values
}


#ifdef DEBUG
int main() {
  unsigned char data[4096];
  int i;
  int readcount;
  unsigned char idbuf[32];
  unsigned char status;

  printf( "Hello! Some initial tests first...\n" );
  
  ad9265_read_byte( AD9265_CHIPID, data );
  printf( "chip ID returned %02x\n", data[0] );

  ad9265_read_byte( AD9265_CHIPGRADE, data );
  printf( "chip grade returned %02x\n", data[0] );

}
#endif
