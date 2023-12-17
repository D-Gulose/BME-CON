#include <stdio.h>

void swap(int* x, int* y)
{
  int tmp = *x;
  *x = *y;
  *y = tmp;
}


int partition(int* A, int l, int r)
{
  // printf("l:%d,r:%d\n", l, r);
  int pivot = A[r]; // lokale variable Ssowas kommt auf stack -> mit size_t am stack im c transformed -> und am assembly stack
  int i = l-1;

  // printf("l:%d", l);

  for (int j = l; j < r; j++)
  {
    // printf("\nj:%d, l:%d,", j, l);
    if (A[j] < pivot)
    {
      i = i+1;
      swap(&A[i], &A[j]);
    }
  }

  i = i+1;
  swap(&A[i], &A[r]);
  // printf("\ni:%d", i);

  return i;
}

void qsort(int* A, int l, int r)
{
  int k;

  if (l < r)
  {
    // printf("\nbefore part:k:%d", k);
    k = partition(A, l, r);
    // printf("\nafter part:k:%d", k);
    // printf("\nleft qsort: k:%d,l:%d, r:%d", k, l, k-1);
    qsort(A, l, k-1);
    // printf("\nright qsort: k:%d, l:%d, r:%d\n", k, k+1, r);
    qsort(A, k+1, r);
  }
}


int input(int *A)
{
  int size;

  if (fscanf(stdin, "%08x\n", &size) != 1)
  {
    return 0;
  }

  for (int i = 0; i < 10; i++)
  {
    if (fscanf(stdin, "%08x\n", A) != 1)
    {
      return size;
    }
    A++;
  }

  return size;
}


void output(int size, int* A)
{
  for (int i = 0; i < size; i++)
  {
    fprintf(stdout, "%08x\n", (unsigned int) A[i]);
  }
}


int main(void)
{
  int A[10]; // array type
  int size;

  size = input(A);
  qsort(A, 0, size-1);
  // printf("\n");
  output(size, A);

  return 0;
}
