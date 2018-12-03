// COMS20001 - Game of Life in XC
// Andrei Bogdan and Zack Dove
// ab_____ and zd17646





//TODO!!!!!
//Change data in & out to not be concurrent - maybe
#include <platform.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <xs1.h>

#define  IMHT 256                  //image height
#define  IMWD 256                  //image width

#define workers 4

#define segHeight (IMHT/workers)+2

#define iterations 100

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

on tile[0] : in port buttons = XS1_PORT_4E; //port for buttons
on tile[0] : out port leds = XS1_PORT_4F;   //port for LEDs

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
//DISPLAYS an LED pattern
int ledManager(out port p, chanend fromDistributor, chanend fromCheckPause) {
    int pattern = 0;    //1st bit...separate green LED : flashing while processing
                        //2nd bit...blue LED : on while exporting
                        //3rd bit...green LED : on while reading
                        //4th bit...red LED : on while paused
    while (1) {
    select {
        case fromCheckPause :> pattern:
            p <: pattern;
            break;
        case fromDistributor :> pattern:
            p <: pattern;
            break;
    }
    }
    return 0;
}

//READ BUTTONS and send button pattern to userAnt
void buttonManager(in port b, chanend toDistributor) {
    int r;
    b when pinseq(14) :> r;
    toDistributor <: 0; //number doesn't matter, dist is just waiting
    while (1) {
        b when pinseq(13)  :> r;
        toDistributor <: r;             // send button pattern to userAnt
    }
}


int recievedExportSignal(chanend toLedManager, chanend fromButtonManager){
    int exportSignal;
    select {
        case fromButtonManager :> exportSignal: //If a signal is received, then export
            printf("export signal recieved.\n");
            toLedManager <: 2;
            return 1;
            break;
        default: //If no signal, then carry on
            return 0;
            break;
    }
    return 0;
}



void DataInStream( chanend c_out)
{
  char infname[] = "256x256.pgm";
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
  printf("DataInStream ready to read in from %s\n", infname);
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[x];
    }
  }
  printf("Completed reading in from %s\n", infname);
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



int modulo(int x , int N){
    return(x % N + N) %N;
}

void worker(int workerID, chanend fromDistributor){
    uchar worldSeg[IMWD/8][segHeight];
    uchar resultByte = 0x00;
    while(1){
        for (int y=0; y<segHeight; y++){
            for (int x = 0; x<IMWD/8; x++){
                fromDistributor :> worldSeg[x][y];
            }
        }
        for (int y=1; y<segHeight-1; y++){
            for (int byte=0; byte<IMWD/8; byte++){
                resultByte = 0x00;
                for (int bit=0; bit<8; bit++){
                    int fertility=0;
                    if (worldSeg[byte][y+1] & (1 << (7-bit))) fertility++; //bits above and below
                    if (worldSeg[byte][y-1] & (1 << (7-bit))) fertility++;
                    if (bit==0){ //edge case for 0th bit (far left)
                        if (worldSeg[modulo(byte-1,IMWD/8)][y-1] & 1) fertility++;
                        if (worldSeg[modulo(byte-1,IMWD/8)][y] & 1) fertility++;
                        if (worldSeg[modulo(byte-1,IMWD/8)][y+1] & 1) fertility++;
                        if (worldSeg[byte][y-1] & 1<<7-1) fertility++;
                        if (worldSeg[byte][y] & 1<<7-1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<7-1) fertility++;
                    }
                    else if (bit == 7){ //case for 7th bit (far right)
                        if (worldSeg[byte][y-1] & 1<<1) fertility++;
                        if (worldSeg[byte][y] & 1<<1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<1) fertility++;
                        if (worldSeg[modulo(byte+1,IMWD/8)][y-1] & 1<<7) fertility++;
                        if (worldSeg[modulo(byte+1,IMWD/8)][y] & 1<<7) fertility++;
                        if (worldSeg[modulo(byte+1,IMWD/8)][y+1] & 1<<7) fertility++;
                    }
                    else { //case where inbetween 1th and 6th bits (only looking in the same byte)
                        if (worldSeg[byte][y-1] & 1<<7-bit-1) fertility++;
                        if (worldSeg[byte][y] & 1<<7-bit-1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<7-bit-1) fertility++;
                        if (worldSeg[byte][y-1] & 1<<7-bit+1) fertility++;
                        if (worldSeg[byte][y] & 1<<7-bit+1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<7-bit+1) fertility++;
                    }
                    if (worldSeg[byte][y] & (1 << (7-bit))){ //alive
                        if (fertility < 2) {
                            resultByte &= ~(1<<7-bit); //die
                        }
                        else if (fertility == 2 || fertility == 3) {
                            resultByte |= 1<<7-bit; //stay alive
                        }
                        else if (fertility > 3) {
                            resultByte &= ~(1<<7-bit); //die
                        }
                    } else { //dead
                        if (fertility == 3) {
                            resultByte |= 1<<7-bit;//come alive
                        }
                        else {
                            resultByte &= ~(1<<7-bit); //stay dead
                        }
                    }
                }
                fromDistributor <: resultByte;
            }
        }
    }
}

void getStartButtonPressed(chanend toButtonManager){
    toButtonManager :> int buttonPressed; //value doesnt matter, just to signal that it's been recieved
}

void getAndPackWorld(uchar packedWorld[IMWD/8][IMHT], chanend c_in){
    uchar pixel;
    for (int y = 0; y<IMHT; y++){
        for (int byte = 0; byte<IMWD/8; byte++){
            packedWorld[byte][y] = 0x00;
            for (int bit=0; bit<8; bit++){
                c_in :> pixel;
                if (pixel == 0xFF) {
                    packedWorld[byte][y] |= 1<<7-bit;
                } else {
                }
            }
        }
    }
}

void unpackAndSendWorld(uchar packedWorld[IMWD/8][IMHT], chanend toPrint){
    uchar pixel;
    for (int y = 0; y<IMHT; y++){
        for (int byte = 0; byte<IMWD/8; byte++){
            for (int bit=0; bit<8; bit++){
                if (packedWorld[byte][y] & (1 << (7-bit))){
                    pixel = 0xFF;
                    toPrint <: pixel;
                } else {
                    pixel = 0x00;
                    toPrint <: pixel;
                }
            }

        }
    }
}

//https://stackoverflow.com/questions/8257714/how-to-convert-an-int-to-string-in-c
void dataOutStream(chanend c_in){
  uchar line[IMWD];
  int res;
  for(int i = 1; i<100; i++){
      //Compile each line of the image and write the image line-by-line
      int length = snprintf( NULL, 0, "%d", i ) + snprintf(NULL, 0, "%s", ".pgm");
      char* outfname = malloc( length + 1 );
      snprintf( outfname, length + 1, "%d", i );
      strcat(outfname, ".pgm");
      res = _openoutpgm( outfname, IMWD, IMHT );
      if (res){
          printf( "DataOutStream: Error opening %s\n.", outfname );
          return;
      }
      for( int y = 0; y < IMHT; y++ ) {
          for(int x = 0; x < IMWD; x++ ) {
              c_in :> line[x];
              printf( "-%4.1d ", line[x]);
          }
          _writeoutline( line, IMWD );
          printf( "End of line\n" );
      }
      _closeoutpgm();
      printf( "Finished printing\n" );
      free (outfname);
  }
}

int numberOfLiveCells(uchar packedWorld[IMWD/8][IMHT]){
    int result = 0;
    for (int y = 0; y<IMHT; y++){
        for (int byte = 0; byte<IMWD/8; byte++){
            for (int bit=0; bit<8; bit++){
                if (packedWorld[byte][y] & (1 << (7-bit))){ //pixel is alive
                    result++;
                }
            }
        }
    }
    return result;
}

unsigned long fixTickOverflow(unsigned long ticks){
    long result;
    unsigned long maximumTicks = (2^32)-1;
    if (ticks < 0) result = (unsigned long) (ticks + maximumTicks); //make it positive, no need to mod
    else result = (unsigned long) (ticks % maximumTicks); //ticks is positive, but we need to mod it so it's not >max ticks
    printf("fix overflow result = %lu\n", result);
    return result;
}

double calculateIterationTime(unsigned long iterationStartTime, unsigned long iterationEndTime){
    double ticksPerSecondMhz = XS1_TIMER_MHZ;
    double ticksPerSecond = ticksPerSecondMhz * 1000000;
    //printf("mhz=%ef, hz=%ef\n", ticksPerSecondMhz, ticksPerSecond);
    double startTimeInSeconds = (double) fixTickOverflow(iterationStartTime)/ticksPerSecond;
    printf("start time  in s= %ef\n", startTimeInSeconds);
    double endTimeInSeconds = (double) fixTickOverflow(iterationEndTime)/ticksPerSecond;
    printf("end time in s = %ef\n", endTimeInSeconds);
    return (double)(endTimeInSeconds - startTimeInSeconds);
}



void timeManager(chanend toDistributor){
    unsigned int overflows = 0;
    unsigned int time = 0;
    unsigned int prevTime = 0;
    unsigned int request = 0;
    timer t;
    while(1){
        //[[ordered]] //Gives the overflow check higher precedence than the distributor request
        select {
            case t when timerafter(time+20000) :> time : //Waits until time >= time+20000, then stores the new time
                    if (time < prevTime) { //If time has gone backwards (due to overflow not a DeLorean)
                        overflows++; //Add 1 to the number of overflows
                    }
                    prevTime = time;
                    break;
            case toDistributor :> request : //If the distributor makes a request
                toDistributor <: prevTime + overflows*UINT_MAX;
                break;
        }
    }
}

double ticksToSeconds(unsigned int ticks){
    double useconds = ticks / (double)XS1_TIMER_MHZ;
    double mseconds = useconds / 1000;
    double seconds  = mseconds / 1000;
    return seconds;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend toPrint, chanend fromAcc, chanend toWorker[workers], chanend toButtonManager, chanend fromCheckPause, chanend toLedManager, chanend toTimeManager){
  //Starting up and wait for tilting of the xCore-200 Explorer

  printf( "ProcessImage: Start, size = %dx%d\n", IMWD, IMHT );
  printf( "Waiting for button press....\n" );
  getStartButtonPressed(toButtonManager);
  printf("Start button pressed, now reading in...\n");
  toLedManager <: 4;
  uchar packedWorld[IMWD/8][IMHT];
  getAndPackWorld(packedWorld, c_in);
  toLedManager <: 1;
  unsigned int currentTime = 0;
  unsigned int startTime = 0;
  toTimeManager <: 1; //Send start signal to time manager
  toTimeManager :> startTime;
  for (int iteration = 0; iteration<=iterations; iteration++){
      for (int i = 0; i<workers; i++){
          int min =(i*IMHT/workers)-1; //used for the extra rows to be passed to workers
          int max =1+(i+1)*IMHT/workers;
          //Send the world segments to the workers
          for (int y= min; y< max; y++){
              for (int x = 0; x < IMWD/8; x++){
                  toWorker[i] <: packedWorld[x][modulo(y,IMHT)];
              }
          }
      }
      //Recieve the world segments back from the workers
      for (int i = 0; i<workers; i++){
          for (int y = i*IMHT/workers; y<(i+1)*IMHT/workers; y++){
              for (int byte = 0; byte < IMWD/8; byte++){
                  toWorker[i] :> packedWorld[byte][y];

              }
          }
      }
      toTimeManager <: 1; //Send request
      toTimeManager :> currentTime;
      int isPaused;
      fromCheckPause :> isPaused;
      if (isPaused){
          //red lighht, rounds, live cells and time elapsed
          toLedManager <: 8; //Send red light
          printf("Paused.\n");
          printf("Current iteration = %i\n",iteration);
          printf("Number of live cells = %i\n",numberOfLiveCells(packedWorld));
          while (1){
              fromCheckPause :> isPaused;
              if (!isPaused) break;
          }
      }
      if (recievedExportSignal(toLedManager, toButtonManager)){
          //export
          unpackAndSendWorld(packedWorld, toPrint);
      }
      toLedManager <: iteration % 2;
      //printf("Round %i completed\n", iteration);
  }
  toTimeManager <: 1; //Send request
  toTimeManager :> currentTime;
  unsigned int ticksTaken = currentTime - startTime;
  double timeTakenS = ticksToSeconds(ticksTaken);
  printf("%i iterations completed in %u ticks or %f seconds\n", iterations, ticksTaken, timeTakenS);
  printf("Mean time taken per iteration = %f\n", timeTakenS/iterations);
  printf( "\nProcessing complete...\n" );
}



void checkPaused(int orientation, chanend toDistributor, chanend c_pauseToLedManager){
    printf("orientation = %i\n", orientation);
    if (orientation <= -50){ //Tilted enough for pause
        //tell workers
        c_pauseToLedManager <: 8;
    } else { //Dont pause
        toDistributor <: 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist, chanend pauseToDistributor, chanend c_pauseToLedManager) {
  i2c_regop_res_t result;
  char status_data = 0;

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
    if (x <= -50){
        pauseToDistributor <: 1;
    } else {
        pauseToDistributor <: 0;
    }

  }
}

// Orchestrate concurrent system and start up all threads
int main(void) {
i2c_master_if i2c[1];               //interface to orientation
chan c_inIO, c_outIO, c_control;    //extend your channel definitions here
chan c_workerToDist[workers];
chan c_distributorToLedManager;
chan c_toButtonManager;
chan c_pauseToDistributor;
chan c_pauseToLedManager;
chan c_timerToDistributor;
par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[0] : orientation(i2c[0],c_control, c_pauseToDistributor, c_pauseToLedManager);        //client thread reading orientation data
    on tile[0] : DataInStream(c_inIO);          //thread to read in a PGM image
    on tile[0] : dataOutStream(c_outIO);       //thread to write out a PGM image
    on tile[0] : ledManager(leds, c_distributorToLedManager, c_pauseToLedManager);
    on tile[0] : buttonManager(buttons, c_toButtonManager);
    on tile[0] : distributor(c_inIO, c_outIO, c_control, c_workerToDist, c_toButtonManager, c_pauseToDistributor, c_distributorToLedManager, c_timerToDistributor);//thread to coordinate work on image
    on tile[0] : timeManager(c_timerToDistributor);
    par (int i=0; i<workers; i++){
        on tile[1] : worker(i, c_workerToDist[i]);
    }
  }

  return 0;
}
