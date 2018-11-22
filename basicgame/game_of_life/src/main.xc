// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width

#define workers 4

#define segHeight (IMHT/workers)+2



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

void worker(int workerID, chanend fromDistributor){
    uchar worldSeg[IMWD][segHeight];
    uchar worldSeg2[IMWD][segHeight];

    for (int y=0; y<segHeight; y++){
        for (int x = 0; x<IMWD; x++){
            fromDistributor :> worldSeg[x][y];
        }
    }
    for (int y=1; y<segHeight-1; y++){
        for (int x=0; x<IMWD; x++){
            int fertility=0;
            if (worldSeg[x][y+1] == 0xFF) fertility++;
            if (worldSeg[x][y-1] == 0xFF) fertility++;
            if (worldSeg[modulo(x+1,IMWD)][y] == 0xFF) fertility++;
            if (worldSeg[modulo(x+1,IMWD)][y+1] == 0xFF) fertility++;
            if (worldSeg[modulo(x+1,IMWD)][y-1] == 0xFF) fertility++;
            if (worldSeg[modulo(x-1,IMWD)][y] == 0xFF) fertility++;
            if (worldSeg[modulo(x-1,IMWD)][y+1] == 0xFF) fertility++;
            if (worldSeg[modulo(x-1,IMWD)][y-1] == 0xFF) fertility++;
            if (worldSeg[x][y] == 0xFF){ //alive
                if (fertility < 2) worldSeg2[x][y] = 0x00; //die
                else if (fertility == 2 || fertility == 3) worldSeg2[x][y] = 0xFF; //chill
                else if (fertility > 3) worldSeg2[x][y] = 0x00; //die
                else worldSeg2[x][y] = worldSeg[x][y];
            } else if (worldSeg[x][y] == 0x00) { //dead
                if (fertility == 3) worldSeg2[x][y] = 0xFF;//come alive
                else worldSeg2[x][y] = worldSeg[x][y];
            }
        }
    }

    for (int y=1; y<segHeigh-1; y++){
        for (int x = 0; x<IMWD; x++){
            fromDistributor <: worldSeg2[x][y];
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
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend toWorker[workers])
{
  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
  fromAcc :> int value;
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
      for( int y = modulo(-1, IMHT); y < 1+(IMHT/workers); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[0] <: world[x][y];
          }
      }
      for( int y = (IMHT/workers)-1; y < 1+(2*IMHT/workers); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[1] <: world[x][y];
          }
      }
      for( int y = (2*IMHT/workers)-1; y < 1+(3*IMHT/workers); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[2] <: world[x][y];
          }
      }
      for( int y = (3*IMHT/workers)-1; y < modulo(1+(4*IMHT/workers),IMHT); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[3] <: world[x][y];
          }
      }
/////////////////////
      for( int y = 0; y < IMHT/workers; y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[0] :> world2[x][y];
          }
      }
      for( int y = IMHT/workers; y < (2*IMHT/workers); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[1] :> world2[x][y];
          }
      }
      for( int y = (2*IMHT/workers); y < (3*IMHT/workers); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[2] :> world2[x][y];
          }
      }
      for( int y = (3*IMHT/workers); y < (4*IMHT/workers); y++) {   //go through all lines
          for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              toWorker[3] :> world2[x][y];
          }
      }
 //////////////////
      //copy world2 to world
      for( int y = 0; y < IMHT; y++ ) {   //go through all lines
            for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                world[x][y] = world2[x][y];
                c_out <: (uchar)( world[y][x]);
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
    distributor(c_inIO, c_outIO, c_control,c_workerToDist);//thread to coordinate work on image
    for (int i=0; i<4; i++){
        worker(i, c_workerToDist[i]);
    }
  }

  return 0;
}
