// COMS20001 - Game of Life in XC
// Andrei Bogdan and Zack Dove
// ab_____ and zd17646

#include <platform.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <xs1.h>

#define  IMHT 16   //image height
#define  IMWD 16 //image width
#define workers 8 //The number of workers
#define iterations 100 //The number of iterations to be completed
#define alwaysExport 0 //Set to 1 to export every iteration
#define alwaysPrint 0  //Set to 1 to also print
#define makeSlower 0 //Set to 1 to intentionally make the processing slower
#define segHeight (IMHT/workers)+2 //The height of the segments to be send to the workers

typedef unsigned char uchar; //Using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;  //Interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

on tile[0] : in port buttons = XS1_PORT_4E; //Port for buttons
on tile[0] : out port leds = XS1_PORT_4F;   //Port for LEDs

#define FXOS8700EQ_I2C_ADDR 0x1E  //Register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

//Reads in from the .pgm file then packs 8 pixels into a single byte
void getAndPackWorld(uchar packedWorld[IMWD/8][IMHT]){
    uchar pixel;
    char infname[] = "16x16e.pgm";
    int res;
    uchar line[ IMWD ];
    printf( "DataInStream: Start...\n" );
    //Open PGM file
    res = _openinpgm( infname, IMWD, IMHT );
    if( res ) {
        printf( "DataInStream: Error opening %s\n.", infname );
        return;
    }
    //Read image line-by-line and send byte by byte to channel c_out
    printf("DataInStream reading in from %s\n", infname);
    for (int y = 0; y<IMHT; y++){
        _readinline( line, IMWD );
        for (int byte = 0; byte<IMWD/8; byte++){
            //printf("byte = %i\n", byte);
            packedWorld[byte][y] = 0x00;
            for (int bit=0; bit<8; bit++){
                //printf("bit = %i\n", bit);
                //printf("byte+bit = %i\n", byte+bit);
                pixel = line[byte*8+bit];
                if (alwaysPrint){
                    printf( "-%4.1d ", pixel);
                }
                if (pixel == 0xFF) {
                    packedWorld[byte][y] |= 1<<7-bit;
                } else {
                }
            }
        }
        if (alwaysPrint){
            printf("\n");
        }
    }
    printf("Completed reading in from %s\n", infname);
    _closeinpgm();
}

//DISPLAYS an LED pattern
int ledManager(out port p, chanend fromDistributor, chanend fromCheckPause) {
    int pattern = 0;    //1st bit...Separate green LED : flashing while processing
                        //2nd bit...Blue LED : on while exporting
                        //3rd bit...Green LED : on while reading
                        //4th bit...Red LED : on while paused
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
    b when pinseq(14) :> r; //SW1 button - pressed to start the process
    toDistributor <: 0; //Send a signal to dist to start
    while (1) {
        b when pinseq(13)  :> r; //SW2 button - pressed to export
        toDistributor <: r; //Send a signal to recievedExportSignal() to trigger exporting
    }
}


//Returns 1 if a signal has been recieved to trigger exporting
int recievedExportSignal(chanend toLedManager, chanend fromButtonManager){
    int exportSignal;
    select {
        case fromButtonManager :> exportSignal: //If a signal is received, then export
            printf("Export signal recieved.\n");
            toLedManager <: 2;
            return 1;
            break;
        default: //If no signal, then carry on
            return 0;
            break;
    }
    return 0;
}


//WAIT function
void waitMoment() {
  timer tmr;
  int waitTime;
  tmr :> waitTime;                       //read current timer value
  waitTime += 40000000;                  //set waitTime to 0.4s after value
  tmr when timerafter(waitTime) :> void; //wait until waitTime is reached
}


//Calculates the modulus, since % behaves incorrectly for negatives
int modulo(int x , int N){
    return(x % N + N) %N;
}

//Worker function that carries out the game of life logic on a segment of the world
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
                    if (worldSeg[byte][y+1] & (1 << (7-bit))) fertility++; //Bits above and below
                    if (worldSeg[byte][y-1] & (1 << (7-bit))) fertility++;
                    if (bit==0){ //Edge case for 0th bit (far left)
                        if (worldSeg[modulo(byte-1,IMWD/8)][y-1] & 1) fertility++;
                        if (worldSeg[modulo(byte-1,IMWD/8)][y] & 1) fertility++;
                        if (worldSeg[modulo(byte-1,IMWD/8)][y+1] & 1) fertility++;
                        if (worldSeg[byte][y-1] & 1<<7-1) fertility++;
                        if (worldSeg[byte][y] & 1<<7-1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<7-1) fertility++;
                    }
                    else if (bit == 7){ //Case for 7th bit (far right)
                        if (worldSeg[byte][y-1] & 1<<1) fertility++;
                        if (worldSeg[byte][y] & 1<<1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<1) fertility++;
                        if (worldSeg[modulo(byte+1,IMWD/8)][y-1] & 1<<7) fertility++;
                        if (worldSeg[modulo(byte+1,IMWD/8)][y] & 1<<7) fertility++;
                        if (worldSeg[modulo(byte+1,IMWD/8)][y+1] & 1<<7) fertility++;
                    }
                    else { //Case where inbetween 1th and 6th bits (only looking in the same byte)
                        if (worldSeg[byte][y-1] & 1<<7-bit-1) fertility++;
                        if (worldSeg[byte][y] & 1<<7-bit-1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<7-bit-1) fertility++;
                        if (worldSeg[byte][y-1] & 1<<7-bit+1) fertility++;
                        if (worldSeg[byte][y] & 1<<7-bit+1) fertility++;
                        if (worldSeg[byte][y+1] & 1<<7-bit+1) fertility++;
                    }
                    if (worldSeg[byte][y] & (1 << (7-bit))){ //Alive
                        if (fertility < 2) {
                            resultByte &= ~(1<<7-bit); //Die
                        }
                        else if (fertility == 2 || fertility == 3) {
                            resultByte |= 1<<7-bit; //Stay alive
                        }
                        else if (fertility > 3) {
                            resultByte &= ~(1<<7-bit); //Die
                        }
                    } else { //Dead
                        if (fertility == 3) {
                            resultByte |= 1<<7-bit;//Come alive
                        }
                        else {
                            resultByte &= ~(1<<7-bit); //Stay dead
                        }
                    }
                }
                fromDistributor <: resultByte;
            }
        }
        if (makeSlower){
            waitMoment();
        }
    }
}

void getStartButtonPressed(chanend toButtonManager){
    toButtonManager :> int buttonPressed; //Value doesnt matter, just to signal that it's been recieved
}


//Unpacks the pixels, and sends to DataStreamOut
void unpackAndSendWorld(uchar packedWorld[IMWD/8][IMHT], chanend toExport, int iteration){
    uchar pixel;
    toExport <: iteration;
    for (int y = 0; y<IMHT; y++){
        for (int byte = 0; byte<IMWD/8; byte++){
            for (int bit=0; bit<8; bit++){
                if (packedWorld[byte][y] & (1 << (7-bit))){
                    pixel = 0xFF;
                    toExport <: pixel;
                } else {
                    pixel = 0x00;
                    toExport <: pixel;
                }
            }

        }
    }
}

//Used https://stackoverflow.com/questions/8257714/how-to-convert-an-int-to-string-in-c
//Outputs data to the screen, and also saves to a file where the file name is a number that increments each time a file is saved
void dataOutStream(chanend c_in){
  uchar line[IMWD];
  int res;
  int iteration;
  while(1){
      //Compile each line of the image and write the image line-by-line
      c_in :> iteration;
      int length = snprintf( NULL, 0, "%d", iteration ) + snprintf(NULL, 0, "%s", ".pgm");
      char* outfname = malloc( length + 1 );
      snprintf( outfname, length + 1, "%d", iteration );
      strcat(outfname, ".pgm");
      res = _openoutpgm( outfname, IMWD, IMHT );
      if (res){
          printf( "DataOutStream: Error opening %s\n.", outfname );
          return;
      }
      printf("Exporting to %s\n", outfname);
      for( int y = 0; y < IMHT; y++ ) {
          for(int x = 0; x < IMWD; x++ ) {
              c_in :> line[x];
              if (alwaysPrint){
                  printf( "-%4.1d ", line[x]); //Print to screen
              }
          }

          _writeoutline( line, IMWD ); //Write to file
          if (alwaysPrint){
              printf("\n"); //End of line
          }
      }
      _closeoutpgm();
      free (outfname);
  }
}

//Counts the number of live cells
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

//Counts how many ticks have passed. And deals with overflows
void timeManager(chanend toDistributor){
    unsigned int overflows = 0;
    unsigned int time = 0;
    unsigned int prevTime = 0;
    unsigned int request = 0;
    timer t;
    while(1){
        [[ordered]] //Gives the overflow check higher precedence than the distributor request
        select {
            case t when timerafter(time+20000) :> time : //Waits until time >= time+20000, then stores the new time
                    if (time < prevTime) { //If time has gone backwards (due to overflow not a DeLorean or Tardis)
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

//Converts ticks to seconds
double ticksToSeconds(unsigned int ticks, unsigned int overflows){
    double useconds = ((double) ticks + (double) overflows*UINT_MAX)/ (double)XS1_TIMER_MHZ;
    double mseconds = useconds / 1000;
    double seconds  = mseconds / 1000;
    return seconds;
}


//Sends segments of the world to the workers threads, and then recieves the processed segments back
void distributor(chanend toPrint, chanend fromAcc, chanend toWorker[workers], chanend toButtonManager, chanend fromCheckPause, chanend toLedManager, chanend toTimeManager){
  printf( "ProcessImage: Start, size = %dx%d\n", IMWD, IMHT );
  printf( "Waiting for button press....\n" );
  getStartButtonPressed(toButtonManager);
  printf("Start button pressed, now reading in...\n");
  toLedManager <: 4;
  uchar packedWorld[IMWD/8][IMHT];
  getAndPackWorld(packedWorld);
  toLedManager <: 1;
  unsigned int prevTime = 0;
  unsigned int currentTime = 0;
  unsigned int overflows = 0;
  unsigned int startTime = 0;
  printf("Processing...\n");
  toTimeManager <: 1; //Send start signal to time manager
  toTimeManager :> startTime;
  for (int iteration = 1; iteration<=iterations; iteration++){
      for (int i = 0; i<workers; i++){
          int min =(i*IMHT/workers)-1; //Used for the extra rows to be passed to workers
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
      if (currentTime < prevTime){
          overflows++;
      }
      prevTime = currentTime;
      int isPaused;
      fromCheckPause :> isPaused;
      if (isPaused){
          //Red lighht, rounds, live cells and time elapsed
          toLedManager <: 8; //Send red light
          printf("Paused.\n");
          printf("Current iteration = %i\n",iteration);
          printf("Number of live cells = %i\n",numberOfLiveCells(packedWorld));
          unsigned int ticksTaken = currentTime - startTime;
          double timeTakenS = ticksToSeconds(ticksTaken, overflows);
          printf("%i iterations completed in %u ticks or %f seconds\n", iteration, ticksTaken, timeTakenS);
          while (1){
              fromCheckPause :> isPaused;
              if (!isPaused) break;
          }
      }
      if (recievedExportSignal(toLedManager, toButtonManager) || alwaysExport || alwaysPrint || iteration==iterations){
          //Export
          unpackAndSendWorld(packedWorld, toPrint, iteration);
      }
      toLedManager <: iteration % 2;
      //printf("Round %i completed\n", iteration);
  }
  toTimeManager <: 1; //Send request
  toTimeManager :> currentTime;
  if (currentTime < prevTime){
      overflows++;
  }
  unsigned int ticksTaken = currentTime - startTime;
  double timeTakenS = ticksToSeconds(ticksTaken, overflows);
  printf("%i iterations completed in %u ticks or %f seconds\n", iterations, ticksTaken, timeTakenS);
  printf("Mean time taken per iteration = %f seconds\n", timeTakenS/iterations);
  printf( "\nProcessing complete...\n" );
}



void checkPaused(int orientation, chanend toDistributor, chanend c_pauseToLedManager){
    printf("orientation = %i\n", orientation);
    if (orientation <= -50){ //Tilted enough for pause
        //Tell distrbutor
        c_pauseToLedManager <: 8;
    } else { //Dont pause
        toDistributor <: 0;
    }
}

// Initialise and  read orientation, send first tilt event to channel
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
    //Check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);
    //Get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);
    if (x <= -50){ //Tilted enough for pause
        pauseToDistributor <: 1;
    } else {
        pauseToDistributor <: 0;
    }

  }
}

//Orchestrate concurrent system and start up all threads
int main(void) {
i2c_master_if i2c[1];               //interface to orientation
chan c_outIO, c_control;    //extend your channel definitions here
chan c_workerToDist[workers];
chan c_distributorToLedManager;
chan c_toButtonManager;
chan c_pauseToDistributor;
chan c_pauseToLedManager;
chan c_timerToDistributor;
par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[0] : orientation(i2c[0],c_control, c_pauseToDistributor, c_pauseToLedManager);        //client thread reading orientation data
    on tile[0] : dataOutStream(c_outIO);       //thread to write out a PGM image
    on tile[0] : ledManager(leds, c_distributorToLedManager, c_pauseToLedManager);
    on tile[0] : buttonManager(buttons, c_toButtonManager);
    on tile[0] : distributor(c_outIO, c_control, c_workerToDist, c_toButtonManager, c_pauseToDistributor, c_distributorToLedManager, c_timerToDistributor);//thread to coordinate work on image
    on tile[0] : timeManager(c_timerToDistributor);
    par (int i=0; i<workers; i++){
        on tile[1] : worker(i, c_workerToDist[i]);
    }
  }

  return 0;
}
