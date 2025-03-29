#include "main.h"

#include <stdio.h>
#include <string.h>

/**************************************************
     Flash module organization(STM32F411)
                               512KBytes

Name        Block base address              size
====      ==================   =======
Sector 0    0x8000000-0x8003FFF           16K bytes
Sector 1    0x8004000-0x8007FFF           16K bytes
Sector 2    0x8008000-0x800BFFF           16K bytes
Sector 3    0x800C000-0x800FFFF           16K bytes
Sector 4    0x8010000-0x801FFFF           64K bytes
Sector 5    0x8020000-0x803FFFF          128K bytes
Sector 6    0x8040000-0x805FFFF          128K bytes
Sector 7    0x8060000-0x807FFFF          128K bytes

******************************************************/

// 0x8060000-0x807FFFF 의 빈공간에 사용자 데이터를 flash programming
// 하도록 설정 한다.
#define ADDR_FLASH_SECTOR7      ((uint32_t) 0x8060000)
#define FLASH_USER_START_ADDR   ((uint32_t) 0x8060000)
#define USER_DATA_ADDRESS        0x8060000
#define FLASH_USER_END_ADDR     ((uint32_t) 0x807FFFF)
#define FLASH_INIT_STATUS       0xFFFFFFFF        // flash 초기 상태
#define FLASH_NOT_INIT_STATUS   0xAAAAAAAA        // flash 초기 상태가 아니다
#define DATA_32                 ((uint32_t) 0x00000001)

HAL_StatusTypeDef flash_write(uint32_t *data32, int size);
HAL_StatusTypeDef flash_write_time(uint32_t *data32, int size);
HAL_StatusTypeDef flash_read(uint32_t *data32, int size);
HAL_StatusTypeDef flash_read_time(uint32_t *data32, int size);
HAL_StatusTypeDef flash_erase();
void flash_main();
void flash_ds1302();
void flash_score_management();
uint32_t flash_read_value=0;






t_st st[8]=
{
		{1234, 1234,{0,0,0},{0,0,0},0,NULL,&st[1]},		//head
		{5678, 5678,{0,0,0},{0,0,0},0,&st[0],&st[2]},
		{1357, 1357,{0,0,0},{0,0,0},0,&st[1],&st[3]},
		{2468, 2468,{0,0,0},{0,0,0},0,&st[2],&st[4]},
		{4680, 4680,{0,0,0},{0,0,0},0,&st[3],&st[5]},
		{3579, 3579,{0,0,0},{0,0,0},0,&st[4],&st[6]},
		{9586, 9586,{0,0,0},{0,0,0},0,&st[5],&st[7]},
		{4564, 4564,{0,0,0},{0,0,0},0,&st[6],NULL}			//tail
};

t_list list =
{
		{&st[0],&st[7]}
};

extern t_ds1302 ds_time;
extern void ds1302_init(void);

void flash_ds1302()
{

	flash_read_value = *(__IO uint32_t *) USER_DATA_ADDRESS;

	if (flash_read_value == FLASH_INIT_STATUS)  // 초기에 아무런 데이터도 존재 하지 않을 경우
	{
		ds_time.magic=0xAAAAAAAA;   // 0101010101010
		ds_time.year = 24;
		ds_time.month = 11;
		ds_time.date = 29;
		ds_time.dayofweek = 6;
		ds_time.hours=12;
		ds_time.minutes = 55;
		ds_time.seconds = 00;
		ds_time.dummy[0] =00;
		ds_time.dummy[1] =00;
		ds_time.dummy[2] =00;


		ds1302_init();
		flash_erase();




		flash_write_time((uint32_t *) &ds_time, sizeof(ds_time));
		flash_write((uint32_t *) &st, sizeof(st));

	}

		else   // 1번 이상 flash memory에 데이터를 write 한 경우
		{
			flash_read_time((uint32_t *) &ds_time, sizeof(ds_time));
			flash_read((uint32_t *) &st, sizeof(st));
		}
}

//void flash_main()
//{
//
//	//t_student *read_student;
//
//	flash_read_value = *(__IO uint32_t *) USER_DATA_ADDRESS;
//
//	if (flash_read_value == FLASH_INIT_STATUS)  // 초기에 아무런 데이터도 존재 하지 않을 경우
//	{
//		flash_erase();
//
//		//student.magic=0x55555555;   // 0101010101010
//		//student.num=1101815;
//		//strncpy((char *)&student.name,"Hong_Gil_Dong", strlen("Hong_Gil_Dong"));
//		student.grade=4.0;
//
//		printf("w magic: %08x\n", student.magic);
//		printf("w num: %08x\n", 	student.num);
//		printf("w name: %s\n", student.name);
//		printf("w grade: %lf\n", student.grade);
//		flash_write((uint32_t *) &student,sizeof(student));
//	}
//	else   // 1번 이상 flash memory에 데이터를 write 한 경우
//	{
//		flash_read((uint32_t *) &student, sizeof(student));
//
//		printf("magic: %08x\n", student.magic);
//		printf("num: %08x\n", 	student.num);
//		printf("name: %s\n", student.name);
//		printf("r grade: %lf\n", student.grade);
//	}
//}


HAL_StatusTypeDef flash_write_time(uint32_t *data32,int size)
{
	uint32_t *mem32 = data32;

  /* Unlock to control */
  HAL_FLASH_Unlock();

  uint32_t Address = FLASH_USER_START_ADDR;  //sector 7의 시작번지

  //printf("size: %d\n", size);

  /* Writing data to flash memory */
  for (int i=0; i < size; )
  {
	  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *mem32) == HAL_OK)
	  {
		  //printf("mem32: %0x\n", *mem32);
		  mem32++;
		  Address = Address + 4; // 0x8060000 -> 0x8060004
		  i += 4;
	  }
	  else
	  {
		  uint32_t errorcode = HAL_FLASH_GetError();
		  return HAL_ERROR;
	  }
  }
  /* Lock flash control register */
  HAL_FLASH_Lock();

  return HAL_OK;
}


HAL_StatusTypeDef flash_write(uint32_t *data32,int size)
{
	uint32_t *mem32 = data32;

  /* Unlock to control */
  HAL_FLASH_Unlock();

  uint32_t Address = FLASH_USER_START_ADDR+16;  //sector 7의 시작번지

  //printf("size: %d\n", size);

  /* Writing data to flash memory */
  for (int i=0; i < size; )
  {
	  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *mem32) == HAL_OK)
	  {
		  //printf("mem32: %0x\n", *mem32);
		  mem32++;
		  Address = Address + 4; // 0x8060000 -> 0x8060004
		  i += 4;
	  }
	  else
	  {
		  uint32_t errorcode = HAL_FLASH_GetError();
		  return HAL_ERROR;
	  }
  }
  /* Lock flash control register */
  HAL_FLASH_Lock();

  return HAL_OK;
}

HAL_StatusTypeDef flash_read_time(uint32_t *data32, int size)
{
//  uint32_t *data32 = ADDR;
  uint32_t address = FLASH_USER_START_ADDR;
  uint32_t end_address = FLASH_USER_START_ADDR +size;

  while(address < end_address)
  {
    *data32 = *(uint32_t*) address;
    data32++;
    address = address + 4;
  }

  return HAL_OK;

}

HAL_StatusTypeDef flash_read(uint32_t *data32, int size)
{
//  uint32_t *data32 = ADDR;
  uint32_t address = 0x08060010;
  uint32_t end_address = address +size;

  while(address < end_address)
  {
    *data32 = *(uint32_t*) address;
    data32++;
    address = address + 4;
  }

  return HAL_OK;

}

HAL_StatusTypeDef flash_erase()
{
	uint32_t SectorError = 0;

	/* Unlock to control */
	HAL_FLASH_Unlock();

	/* Calculate sector index */
	uint32_t UserSector = 7;     // sector 번호
	uint32_t NbOfSectors = 1;    // sector 수

	/* Erase sectors */
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = UserSector;
	EraseInitStruct.NbSectors = NbOfSectors;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		uint32_t errorcode = HAL_FLASH_GetError();
		return HAL_ERROR;
	}

	/* Lock flash control register */
	HAL_FLASH_Lock();

	return HAL_OK;
}
