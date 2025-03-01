/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h> 

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
OSAL_THREAD_HANDLE input_thread;

int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

// Global variables
char user_input;
// osal_mutex_t input_mutex;

// Function to read user input
OSAL_THREAD_FUNC readInput()
{
   while(1){

    printf("DEAR USER YOU MAY CHOOSE : ");
   //  fgets(input, sizeof(input), stdin);

    // Lock the mutex and update the global variable
   //  osal_mutex_lock(&input_mutex);
   //  strcpy(user_input, input);
      user_input = getchar();
    // Discard the trailing newline character from the input buffer
    getchar();
   //  osal_mutex_unlock(&input_mutex);
   printf("THE USER HAS CHOSEN: %c\n", user_input);
   //  osal_thread_exit(NULL);
   }
}


void get_input_int16(uint16 slave_no,uint8 module_index, uint16 *val)
{
   uint8 *data_ptr;
   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].inputs;
   *val = 0;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
    * addresses
    */
   *val |= ((*data_ptr++) & 0xFF);
   *val |= ((*data_ptr++ >> 8) & 0xff);

   return;
}

void get_input_int32(uint16 slave_no,uint8 module_index, int32 *val)
{
   uint8 *data_ptr;
   *val = 0;
   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].inputs;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
    * addresses
    */
   *val |= ((*data_ptr++) & 0xFF);
   *val |= ((*data_ptr++ << 8) & 0xff00);
   *val |= ((*data_ptr++ << 16) & 0xff0000);
   *val |= ((*data_ptr++ << 24) & 0xff000000);

   // printf("VAL IS %d", *val);

   return;
}

void get_output_int32(uint16 slave_no,uint8 module_index, int32 *val)
{
   uint8 *data_ptr;
   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].outputs;
   *val = 0;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
    * addresses
    */
   *val |= ((*data_ptr++) & 0xFF);
   *val |= ((*data_ptr++ << 8) & 0xff00);
   *val |= ((*data_ptr++ << 16) & 0xff0000);
   *val |= ((*data_ptr++ << 24) & 0xff000000);

      // printf("VAL IS %d", *val);


   return;
}


void get_output_int16(uint16 slave_no,uint8 module_index, uint16 *val)
{
   uint8 *data_ptr;

   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].outputs;
   *val = 0;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can handle misaligned
    * addresses
    */
   *val |= ((*data_ptr++) & 0xFF);
   *val |= ((*data_ptr++ << 8) & 0xff);


   return;
}


void set_output_int16 (uint16 slave_no, uint8 module_index, int16 value)
{
   uint8 *data_ptr;
   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].outputs;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can handle misaligned
    * addresses
    */
   *data_ptr++ = (value >> 0) & 0xFF;
   *data_ptr = (value >> 8) & 0xFF;
}

void set_output_int32 (uint16 slave_no, uint8 module_index, int32 value)
{
   uint8 *data_ptr;
   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].outputs;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can handle misaligned
    * addresses
    */
   *data_ptr++ = (value >> 0) & 0xFF;
   *data_ptr++ = (value >> 8) & 0xFF;
   *data_ptr++ = (value >> 16) & 0xFF;
   *data_ptr = (value >> 24) & 0xFF;
}


void set_output_bit (uint16 slave_no, uint8 module_index, uint8 value)
{
   /* Get the the startbit position in slaves IO byte */
   uint8 startbit = ec_slave[slave_no].Ostartbit;
   /* Set or Clear bit */
   if (value == 0)
      *ec_slave[slave_no].outputs &= ~(1 << (module_index - 1 + startbit));
   else
      *ec_slave[slave_no].outputs |= (1 << (module_index - 1 + startbit));
}


void simpletest(char *ifname)
{
    // int i, j, oloop, iloop, chk;
    int i, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
   {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */


        if ( ec_config_init(FALSE) > 0 )
        {
        printf("%d slaves found and configured.\n",ec_slavecount);

        if (forceByteAlignment)
        {
        ec_config_map_aligned(&IOmap);
        }
        else
        {
        int used_mem;    
        used_mem = ec_config_map(&IOmap);
        printf("SOEM IOMap size: %d\n", used_mem);
        }
        // for (int i = 1; i <= ec_slavecount; i++) {
        //     ec_slave[i].blockLRW = 1; /* blocking LRW command */
        // }
        ec_configdc();

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            /* Print som information on the mapped network */
            for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                        cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                        ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                // printf(" Configured address: %u\n", ec_slave[cnt].configadr);
                // printf(" Outputs address: %hhn\n", ec_slave[cnt].outputs);
                // printf(" Inputs address: %hhn\n", ec_slave[cnt].inputs);

            // for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
            // {
            //     printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
            //             (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
            //             ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
            //             ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
            // }
            // printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                    // ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);

            }
            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            set_output_int16(1, (0xE - 0xE) >> 1, 1); // CTL0 on
            set_output_int16(1, (0x12 - 0xE) >> 1, 2063); // 0000000000001010 start, servo on

            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;

            
            /* wait for all slaves to reach OP state */
            do
            {
            //  set_output_int16(1, (0xE - 0xE) >> 1, 1); // CTL0 on
            // set_output_int16(1, (0x12 - 0xE) >> 1, 6); // 0000000000001010 start, servo on
   
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            osal_usleep(5000);
            // uint16 speed = 100;

            set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
            set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 3); // absolute position movement
            // set_output_int16(1, (0x14 - 0xE) >> 1, speed); // 10 % speed
            // set_output_int32(1, (0x18 - 0xE) >> 1, 3000); // move to position 200            
            set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 3); // 0000000000000110 alarm reset, servo on
            
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            osal_usleep(5000);          

            uint16 val =0;
            
            uint16 alarm = 0;
            uint16 movespeed = 0;

            int32 setpose = 0;
            int32 enc = 0;
            int i = 0;
            uint16 sys0 = 0;
            uint16 sys1 = 0;
            // int position = 1000;
            int32 position = 0;
            // speed = 100;
            // int j =0;
            while(i < 100000){

               get_input_int16(1,(0x4 - 0x0 )>> 1, &val);
               // printf("VAL %d\n", val );

               get_input_int16(1,(0x0000 - 0x0 )>> 1, &sys0);
               // printf("SYS0 %d\n", sys0 );

               get_input_int16(1,(0x0002 - 0x0 )>> 1, &sys1);
               // printf("SYS1 %d\n", sys1 );
                  // printf("ENCODER POSITION %d\n", enc );

               get_input_int16(1,(0x6 - 0x0 )>> 1, &alarm);
               // printf("alarm %d\n", alarm );

               get_output_int16(1,(0xE - 0xE )>> 1, &movespeed);
               // printf("movespeed %u\n", movespeed );

               get_output_int32(1,(0x18 - 0xE )>> 1, &setpose);
               // printf("SETPOSE %d\n", setpose );

               get_input_int32(1, (0x8 - 0x0) >> 1, &enc);
               // printf("ENCODER POSITION %d\n", enc );
               // printf("USER INPUT %c \n", user_input);
                  
               
               if(user_input =='y' && val >= 14){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 11); // absolute position movement
                  set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 100 % speed
                  // set_output_int32(1, (0x18 - 0xE) >> 1, position); // move to position 1000            
                  // printf("INSTATE Y \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  user_input = 'i';

                  position += 100;
                  }

                   if(user_input =='a' && val >= 14){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 1); // absolute position movement
                  set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 100 % speed
                  set_output_int32(1, (0x18 - 0xE) >> 1, 2000); // move to position 1000 
                  position = 2000;           
                  // printf("INSTATE A \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  user_input = 'l';

                  }

                  if(user_input =='b' && val >= 14){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 1); // absolute position movement
                  set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 100 % speed
                  set_output_int32(1, (0x18 - 0xE) >> 1, 1000); // move to position 1000
                  position = 1000;            
                  // printf("INSTATE B \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  user_input = 'i';

                  }
 
 

                if(user_input =='h' && val >= 14){
                  // set_output_int16(1, (0xE - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 1); // absolute position movement
                  set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
                  // set_output_int32(1, (0x18 - 0xE) >> 1, 1000); // move to position 200            
                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 3); // 0000000000000010 start, servo on
                  // printf("HOMING \n");
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  user_input = 'i';

                  
                  }     
               
               if(user_input =='o' && val >= 14){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 1); // absolute position movement
                  // set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
                  set_output_int32(1, (0x18 - 0xE) >> 1, 3000); // move to position 200            
                  printf("IN STATE N \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  // user_input = 'i';


                  }

               if(user_input =='n' && val >= 14){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 12); // absolute position movement
                  // set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
                  set_output_int32(1, (0x18 - 0xE) >> 1, 4000); // move to position 200            
                  printf("IN STATE N \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  user_input = 'i';


                  }
             


               if(user_input =='q' ){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 8); // absolute position movement
                  // set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
                  // set_output_int32(1, (0x18 - 0xE) >> 1, 4000); // move to position 200            
                  printf("IN STATE N \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  // user_input = 'i';


                  }
               if(user_input =='r' && val >= 14){
                  // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 3); // absolute position movement
                  set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
                  // set_output_int32(1, (0x18 - 0xE) >> 1, 2000); // move to position 200            
                  printf("IN STATE N \n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  user_input = 'i';

                  }

               if(user_input =='t' && val >= 14){
               // set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
               set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 2); // absolute position movement
               set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
               // set_output_int32(1, (0x18 - 0xE) >> 1, 2000); // move to position 200            
               printf("IN STATE N \n");

               set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               osal_usleep(5000);
               user_input = 'i';


               }


                if(user_input =='i' && val >= 14){
               set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
               set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 14); // absolute position movement
               // set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
               // set_output_int32(1, (0x18 - 0xE) >> 1, 2000); // move to position 200            
               // printf("INSTATE I \n");

               set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               osal_usleep(5000);
               if(enc == position)
               {
               user_input = 'a';
               }

               }

                if(user_input =='l' && val >= 14){
                  set_output_int16(1, (0x10 - 0xE) >> 1, 1); // CTL0 on
                  set_output_int16(1, (0x16 - 0xE) >> 1, (uint16) 14); // absolute position movement
                  // set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
                  // set_output_int32(1, (0x18 - 0xE) >> 1, 2000); // move to position 200            
                  // printf("INSTATE L\n");

                  set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
                  ec_send_processdata();
                  wkc = ec_receive_processdata(EC_TIMEOUTRET);
                  osal_usleep(5000);
                  if(enc == position)
                  {
                  user_input = 'b';
                  }
               }

               else{
               set_output_int16(1, (0x12 - 0xE) >> 1, (uint16) 2); // 0000000000000010 start, servo on
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               osal_usleep(5000);
               }
            }

        }   
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
    //  pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);

      osal_thread_create(&input_thread, 128000, &readInput, (void*) &ctime);

      /* start cyclic part */
        printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

        simpletest(argv[1]);


//    task_spawn ("simpletest", simpletest, 9, 8192, NULL);
   }
   else
   {
      ec_adaptert * adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }

   printf("End program\n");
   return (0);
}
