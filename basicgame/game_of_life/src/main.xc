// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width

int workers = 4;

typedef unsigned char uchar;      //using uchar as shorthand

port p_scl = XS1_PORT_1E;         //interface ports to orientation
port p_sda = XS1_PORT_1F;

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{
  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

  //Open PGM file
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s\n.", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[x];
      printf( "-%4.1d ", line[ x ] ); //show image values
    }
    printf( "\n" );
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  return;
}

//WAIT function
void waitMoment() {
  timer tmr;
  int waitTime;
  tmr :> waitTime;                       //read current timer value
  waitTime += 40000000;                  //set waitTime to 0.4s after value
  tmr when timerafter(waitTime) :> void; //wait until waitTime is reached
}

void checkCellsAround(){

}
int modulo(int x , int N){
    return(x % N + N) %N;
}

void worker(int workerID, channend fromDistributor){
    segHeight = (IMHHT/4)+2;
    uchar worldSeg[IMWD][segHeight];
    for (int y=0; y<segHeight; y++){
        for (int x = 0; x<IMWD; x++){
            fromDistributor :> worldSeg[x][y];
        }
    }
    for (int y=0; y<segHeight; y++){
        for (int x=0; x<IMWD; x++){
            fertility=0;
            fertility = 0;
            if (world[x][modulo(y+1,segHeight)] == 0xFF) fertility++;
            if (world[x][modulo(y-1,segHeight)] == 0xFF) fertility++;
            if (world[modulo(x+1,IMWD)][y] == 0xFF) fertility++;
            if (world[modulo(x+1,IMWD)][modulo(y+1,segHeight)] == 0xFF) fertility++;
            if (world[modulo(x+1,IMWD)][modulo(y-1,segHeight)] == 0xFF) fertility++;
            if (world[modulo(x-1,IMWD)][y] == 0xFF) fertility++;
            if (world[modulo(x-1,IMWD)][modulo(y+1,segHeight)] == 0xFF) fertility++;
            if (world[modulo(x-1,IMWD)][modulo(y-1,segHeight)] == 0xFF) fertility++;
            if (world[x][y] == 0xFF){ //alive
                if (fertility < 2) world2[x][y] = 0x00; //die
                else if (fertility == 2 || fertility == 3) world2[x][y] = 0xFF; //chill
                else if (fertility > 3) world2[x][y] = 0x00; //die
                else world2[x][y] = world[x][y];
            } else if (world[x][y] == 0x00) { //dead
                if (fertility == 3) world2[x][y] = 0xFF;//come alive
                else world2[x][y] = world[x][y];
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, channend toWorker)
{
  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
  fromAcc :> int value;
  int fertility;
  uchar world[IMHT][IMWD];
  //Read in and do something with your image values..
  //This just inverts every pixel, but you should
  //change the image according to the "Game of Life"
  printf( "Processing...\n" );
  for( int y = 0; y < IMHT; y++ ) {     //go through all lines
      for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
          c_in :> world[y][x];          //read the pixel value
      }
  }
  uchar world2[IMHT][IMWD];
  for (int iteration = 0; iteration<10; iteration++){
      for( int y = 0; y < IMHT; y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[i] <: world[x][y];
          }
      }
      //copy world2 to world
      for( int y = 0; y < IMHT; y++ ) {   //go through all lines
            for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                world[x][y] = world2[x][y];
                c_out <: (uchar)( world[y][x]); //send some modified pixel out
            }
        }
      waitMoment();
  }
  printf( "\nOne processing round completed...\n" );
  waitMoment();
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(chanend c_in)
{
  uchar line[IMWD];

  //Compile each line of the image and write the image line-by-line
  while(1){
      for( int y = 0; y < IMHT; y++ ) {
        for(int x = 0; x < IMWD; x++ ) {
          c_in :> line[x];
          printf( "-%4.1d ", line[ x ] );
        }

        printf( "DataOutStream: Line written...\n" );
      }
  }
  printf( "DataOutStream: Done...\n" );
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;
  int tilted = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }
  
  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {

    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to distributor after first tilt
    if (!tilted) {
      if (x>30) {
        tilted = 1 - tilted;
        toDist <: 1;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

i2c_master_if i2c[1];               //interface to orientation

char infname[] = "test.pgm";     //put your input image path here
chan c_inIO, c_outIO, c_control;    //extend your channel definitions here
chan c_workerToDist[workers];
uchar world[IMWD][IMHT];
printf("mod thing: %i /n ", modulo(-1,64));
par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    orientation(i2c[0],c_control);        //client thread reading orientation data
    DataInStream(infname, c_inIO);          //thread to read in a PGM image
    DataOutStream(c_outIO);       //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control);//thread to coordinate work on image
    for (int i=0; i<4; i++){
        worker(i, c_workerToDist);
    }
  }

  return 0;
}
