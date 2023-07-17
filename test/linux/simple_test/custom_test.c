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
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

typedef struct PACKED
{
    // uint8_t timepotato;
    uint16_t   Toyo_CTL0;
    uint16_t Toyo_CTL1;
    bool      Toyo_ORG;
    bool      Toyo_SERVO_ON;
    bool      Toyo_ALARM_RESET;
    bool      Toyo_START;
    bool      Toyo_PRG0;
    bool      Toyo_PRG1;
    bool      Toyo_PRG2;
    bool      Toyo_PRG3;
    bool      Toyo_PRG4;
    bool      Toyo_PRG5;
    bool      Toyo_PRG6;
    bool      Toyo_ORGSIG;
    bool      Toyo_Revers1;
    bool      Toyo_Revers2;
    bool      Toyo_Revers3;
    bool      Toyo_Revers4;
    uint16_t   Toyo_MovSpeedSet;
    uint16_t   Toyo_MovType;
    uint32_t    Toyo_ABSamount;
} out_TOYO_t;

typedef struct PACKED
{

    // uint8_t timepass;
    uint16_t   Toyo_SYS0;
    uint16_t   Toyo_SYS1;
    bool     Toyo_ORG_S;
    bool     Toyo_INP;
    bool     Toyo_READY;
    bool     Toyo_SERVO_S;
    bool     Toyo_PRG_0S;
    bool     Toyo_PRG_1S;
    bool     Toyo_PRG_2S;
    bool     Toyo_PRG_3S;
    bool     Toyo_PRG_4S;
    bool     Toyo_PRG_5S;
    bool     Toyo_Rever1;
    bool     Toyo_Rever2;
    bool     Toyo_Rever3;
    bool     Toyo_Rever4;
    bool     Toyo_Rever5;
    bool     Toyo_Rever6;
    uint16_t   Toyo_AlarmStatus;
    uint32_t    Toyo_EcdPos;

} in_TOYO_t;

in_TOYO_t slave_TOYO_1;
in_TOYO_t slave_TOYO_2;


// int32 get_input_int32(uint16 slave_no,uint8 module_index)
// {
//    int32 return_value;
//    uint8 *data_ptr;
//    /* Get the IO map pointer from the ec_slave struct */
//    data_ptr = ec_slave[slave_no].inputs;
//    /* Move pointer to correct module index */
//    data_ptr += module_index * 2;
//    /* Read value byte by byte since all targets can't handle misaligned
//     * addresses
//     */
//    return_value = *data_ptr++;
//    return_value += (*data_ptr++ << 8);
//    return_value += (*data_ptr++ << 16);
//    return_value += (*data_ptr++ << 24);

//    return return_value;
// }

void get_input_int16(uint16 slave_no,uint8 module_index, int16 *val)
{
   uint8 *data_ptr;
   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].inputs;
   /* Move pointer to correct module index */
   data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
    * addresses
    */
   *val |= ((*data_ptr++) & 0xFF);
   *val |= ((*data_ptr++ << 8) & 0xff);

   return;
}

void get_input_int32(uint16 slave_no,uint8 module_index, int32 *val)
{
   uint8 *data_ptr;
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

   return;
}

void get_output_int32(uint16 slave_no,uint8 module_index, int32 *val)
{
   uint8 *data_ptr;
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

   return;
}
// void set_input_int32 (uint16 slave_no, uint8 module_index, int32 value)
// {
//    uint8 *data_ptr;
//    /* Get the IO map pointer from the ec_slave struct */
//    data_ptr = ec_slave[slave_no].inputs;
//    /* Move pointer to correct module index */
//    data_ptr += module_index * 2;
//    /* Read value byte by byte since all targets can handle misaligned
//     * addresses
//     */
//    *data_ptr++ = (value >> 0) & 0xFF;
//    *data_ptr++ = (value >> 8) & 0xFF;
//    *data_ptr++ = (value >> 16) & 0xFF;
//    *data_ptr++ = (value >> 24) & 0xFF;
// }

// uint8 get_input_bit (uint16 slave_no,uint8 module_index)
// {
//    /* Get the the startbit position in slaves IO byte */
//    uint8 startbit = ec_slave[slave_no].Istartbit;
//    /* Mask bit and return boolean 0 or 1 */
//    if (*ec_slave[slave_no].inputs & BIT (module_index - 1  + startbit))
//       return 1;
//    else
//       return 0;
// }

void get_output_int16(uint16 slave_no,uint8 module_index, int16 *val)
{
   uint8 *data_ptr;

   /* Get the IO map pointer from the ec_slave struct */
   data_ptr = ec_slave[slave_no].outputs;
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
// uint8 get_output_bit (uint16 slave_no,uint8 module_index)
// {
//    /* Get the the startbit position in slaves IO byte */
//    uint8 startbit = ec_slave[slave_no].Ostartbit;
//    /* Mask bit and return boolean 0 or 1 */
//    if (*ec_slave[slave_no].outputs & BIT (module_index - 1  + startbit))
//       return 1;
//    else
//       return 0;
// }

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

// switch(statusword){
//        case (Not_ready_to_switch_on): {
//      		/* Now the FSM should automatically go to Switch_on_disabled*/
//                        break;
//                }
//                case (Switch_on_disabled): {
//                         /* Automatic transition (2)*/
//                        controlword = 0;
//                        controlword |= (1 << control_enable_voltage)
//                        				| (1 << control_quick_stop);
//                        break;
//                }
//                case (Ready_to_switch_on): {
//                        /* Switch on command for transition (3) */
//                        controlword |= 1 << control_switch_on;
//                        break;
//                }
//                case (Switch_on): {
//                        /* Enable operation command for transition (4) */
//                        controlword |= 1 << control_enable_operation;
//                        break;
//                }
//                case (Operation_enabled): {
//                        /* Setting modes of operation
//                                * Value Description
//                                -128...-2 Reserved
//                                -1 No mode
//                                0 Reserved
//                                1 Profile position mode
//                                2 Velocity (not supported)
//                                3 Profiled velocity mode
//                                4 Torque profiled mode
//                                5 Reserved
//                                6 Homing mode
//                                7 Interpolated position mode
//                                8 Cyclic Synchronous position
//                                ...127 Reserved*/
                               
//                        uint16_t mode = 8; /* Setting Cyclic Synchronous position */
//                        int mode_size = sizeof(mode);
//                        SDO_result = ec_SDOwrite(SLAVE_NB, MODES_OF_OPERATION_INDEX, 0,
//                                                 0, mode_size, &mode, EC_TIMEOUTRXM);
//                        break;
//                }
//                case (Quick_stop_active): {
//                        break;
//                }
//                case (Fault_reaction_active): {
//                        break;
//                }
//                case (Fault): {
//                        /* Returning to Switch on Disabled */
//                        controlword = (1 << control_fault_reset);
//                        break;
//                }
//                default:{
//                        OSEE_PRINT("Unrecognized status\n");
//                        break;}
//        }


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
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;

            
            /* wait for all slaves to reach OP state */
            do
            {
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

            set_output_int16(1, (0xE - 0xE) >> 1, 1); // CTL0 on
            set_output_int16(1, (0x16 - 0xE) >> 1, 2); // absolute position movement
            set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
            // set_output_int32(1, (0x18 - 0xE) >> 1, 1000); // move to position 200            
            set_output_int16(1, (0x12 - 0xE) >> 1, 7); // 0000000000001010 start, servo on

            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            osal_usleep(500000);

            printf("FINISHED HOMING TYPE THING OR SOMETHING IDK");


            set_output_int16(1, (0x16 - 0xE) >> 1, 1); // absolute position movement
            set_output_int16(1, (0x14 - 0xE) >> 1, 100); // 10 % speed
            set_output_int32(1, (0x18 - 0xE) >> 1, 1000); // move to position 200            
            set_output_int16(1, (0x12 - 0xE) >> 1, 6); // 0000000000001010 start, servo on
 
            int16 val =0;
            
            int16 alarm = 0;
            int16 movespeed = 0;

            int32 setpose = 0;
            int32 enc = 0;
            int i = 0;
            while(i < 700){

           
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

               printf("WKC %d\n", wkc );

               get_input_int16(1,(0x4 - 0x0 )>> 1, &val);
               printf("POTATO %d\n", val );

               get_input_int16(1,(0x6 - 0x0 )>> 1, &alarm);
               printf("alarm %d\n", alarm );

               get_output_int16(1,(0x14 - 0xE )>> 1, &movespeed);
               printf("movespeed %d\n", movespeed );

               get_output_int32(1,(0x18 - 0xE )>> 1, &setpose);
               printf("SETPOSE %d\n", setpose );



               get_input_int32(1, (0x8 - 0x0) >> 1, &enc);
               printf("ENCODER POSITION %d\n", enc );

               osal_usleep(5000);
               i++;
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
