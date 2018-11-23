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

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

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
void DataInStream( chanend c_out)
{
  char infname[] = "test.pgm";
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

void bitPackWorld(uchar unpackedWorld[IMWD][IMHT], chanend toDistributor){
    uchar packedWorld[IMWD/8][IMHT];
    for (int y = 0; y<IMHT; y++){
        for (int byte = 0; byte<IMWD/8; byte++){
            packedWorld[byte][y] = 0x00;
            for (int bit=0; bit<8; bit++){
                if (unpackedWorld[byte+bit][y] == 0xFF) {
                    packedWorld[byte][y] = packedWorld[byte][y] |= 1<<7-bit;
                }
            }
        }
    }
    toDistributor <: packedWorld;

}

//WAIT function
void waitMoment() {
  timer tmr;
  int waitTime;
  tmr :> waitTime;                       //read current timer value
  waitTime += 40000000;                  //set waitTime to 0.4s after value
  tmr when timerafter(waitTime) :> void; //wait until waitTime is reached
}



int modulo(int x , int N){
    return(x % N + N) %N;
}

void worker(int workerID, chanend fromDistributor){
    uchar worldSeg[IMWD][segHeight];
    uchar worldSeg2[IMWD][segHeight];
    printf("workerID%i\n",workerID);
    while(1){
        printf("a\n");
        for (int y=0; y<segHeight; y++){
            for (int x = 0; x<IMWD; x++){
                fromDistributor :> worldSeg[x][y];
            }
        }
        printf("b\n");
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
        printf("c\n");
        for (int y=1; y<segHeight-1; y++){
            for (int x = 0; x<IMWD; x++){
                fromDistributor <: worldSeg2[x][y];
            }
        }
        printf("d\n");
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
          c_in :> world[x][y];          //read the pixel value
      }
  }
  chanend fromBitPacker;
  bitPackWorld(world, fromBitpacker);
  fromBitPacker :> uchar packedWorld[IMWD/8][IMHHT];
  printf("1\n");
  uchar world2[IMHT][IMWD];
  for (int iteration = 0; iteration<10; iteration++){
      printf("2\n");
      for (int i = 0; i<workers; i++){
          int min =(i*IMHT/workers)-1;
          int max =1+(i+1)*IMHT/workers;
          printf("i = %i , min = %i, max = %i\n",i,min,max );
          for (int y= min; y< max; y++){
              for (int x = 0; x < IMWD; x++){
                  toWorker[i] <: world[x][modulo(y,IMHT)];
              }
          }
      }
      printf("3\n");
/////////////////////
      for (int i = 0; i<workers; i++){
          for (int y = i*IMHT/workers; y<(i+1)*IMHT/workers; y++){
              for (int x = 0; x < IMWD; x++){
                  toWorker[i] :> world2[x][y];
              }
          }
      }
      printf("9\n");
 ///////////////////
      //copy world2 to world
      for( int y = 0; y < IMHT; y++ ) {   //go through all lines
            for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
                world[x][y] = world2[x][y];
                c_out <: (uchar)( world[y][x]);
            }
      }
      printf("10\n");


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

     //put your input image path here
chan c_inIO, c_outIO, c_control;    //extend your channel definitions here
chan c_workerToDist[workers];


par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[0] : orientation(i2c[0],c_control);        //client thread reading orientation data
    on tile[0] : DataInStream(c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream(c_outIO);       //thread to write out a PGM image
    on tile[0] : distributor(c_inIO, c_outIO, c_control, c_workerToDist);//thread to coordinate work on image
    par (int i=0; i<workers; i++){
        on tile[1] : worker(i, c_workerToDist[i]);
    }
  }

  return 0;
}
