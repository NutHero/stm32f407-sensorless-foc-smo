#include "./BSP/SORT/sort.h"

/*
 功能：冒泡排序
 注：排序规则，数据从右到左，值逐渐减小
 pdat : 指向要排序的数据，
 len : 数据的长度
 */
void bubble_sort(uint16_t *pdat, uint8_t len)
{
	uint16_t i, j;
	uint16_t temp;
	for (i = 0; i < len - 1; i++)
	{
		for (j = 0; j < len - i - 1; j++)
		{
			if (pdat[j] > pdat[j + 1])//比较相邻两个数的大小,如果前一个数比后一个数大，则两个数交换顺序，并判断下一组数,否则直接判断下一组数
			{
				temp = pdat[j];
				pdat[j] = pdat[j + 1];
				pdat[j + 1] = temp;
			}		
		}
	}
}

