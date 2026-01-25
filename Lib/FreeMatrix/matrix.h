/* 矩阵运算库头文件 */

#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdlib.h> 
#include <time.h>
#include <string.h> 



//Macro Define Section
#define _IN
#define _OUT
#define _IN_OUT
#define MAX(x,y) (x)>(y)?(x):(y)
#define MIN(x,y) (x)<(y)?(x):(y)

#define _CRT_SECURE_NO_WARNINGS
#define PI 3.14159265358979323846
#define POSITIVE_INFINITY 999999999
#define NEGATIVE_INFINITY -999999999
#define _ERROR_NO_ERROR                                             0x00000000    //无错误
#define _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY                       0x00000001    //分配堆内存失败
#define _ERROR_SVD_EXCEED_MAX_ITERATIONS                            0x00000002    //svd超过最大迭代次数
#define _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL                     0x00000003    //矩阵行数或列数不相等
#define _ERROR_MATRIX_MULTIPLICATION                                0x00000004    //矩阵乘法错误(第一个矩阵列数不等于第二个矩阵的行数)
#define _ERROR_MATRIX_MUST_BE_SQUARE                                0x00000005    //矩阵必须为方阵
#define _ERROR_MATRIX_NORM_TYPE_INVALID                             0x00000006    //矩阵模类型无效
#define _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS                     0x00000007    //矩阵方程无解
#define _ERROR_MATRIX_EQUATION_HAS_INFINTY_MANNY_SOLUTIONS          0x00000008    //矩阵方程有无穷多解
#define _ERROR_QR_DECOMPOSITION_FAILED                              0x00000009    //QR分解失败
#define _ERROR_CHOLESKY_DECOMPOSITION_FAILED                        0x0000000A    //cholesky分解失败
#define _ERROR_IMPROVED_CHOLESKY_DECOMPOSITION_FAILED               0x0000000B    //improved cholesky分解失败
#define _ERROR_LU_DECOMPOSITION_FAILED                              0x0000000C    //LU分解失败
#define _ERROR_CREATE_MARTIX_FAILED                                 0x0000000D    //创建矩阵失败
#define _ERROR_MATRIX_TRANSPOSE_FAILED                              0x0000000E    //矩阵转置失败
#define _ERROR_CREATE_VECTOR_FAILED                                 0x0000000F    //创建向量失败
#define _ERROR_VECTOR_DIMENSION_NOT_EQUAL                           0x00000010    //向量维数不一致
#define _ERROR_VECTOR_NORM_TYPE_INVALID                             0x00000011    //向量模类型无效
#define _ERROR_VECTOR_CROSS_FAILED                                  0x00000012    //向量叉乘失败
#define _ERROR_INPUT_PARAMETERS_ERROR                               0x00010000    //输入参数错误

typedef unsigned int ERROR_ID;
typedef int INDEX;
typedef short FLAG;
typedef int INTEGER;
typedef double REAL;
typedef char* STRING;
typedef void VOID;

typedef struct matrix
{
    INTEGER rows;
    INTEGER columns;
    REAL* p;
} MATRIX;

typedef struct matrix_node
{
    MATRIX* ptr;
    struct matrix_node* next;
} MATRIX_NODE;

typedef struct matrix_element_node
{
    REAL* ptr;
    struct matrix_element_node* next;
} MATRIX_ELEMENT_NODE;

typedef struct stacks
{
    MATRIX_NODE* matrixNode;
    MATRIX_ELEMENT_NODE* matrixElementNode;
} STACKS;

//初始栈
VOID init_stack(_IN_OUT STACKS* S);

//释放栈
VOID free_stack(_IN STACKS* S);

float** Matrix_Jac_Eig(float **array, int n, float *eig);
int Matrix_Free(float **tmp, int m, int n);
ERROR_ID EigenValueVecter(_IN MATRIX* A, _OUT MATRIX* B, _OUT MATRIX* C);

void print_matrix(MATRIX* a, STRING string);
void matrix_test();

// 创建矩阵 返回矩阵指针
MATRIX* creat_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// 创建多个矩阵 返回矩阵指针 个数count
MATRIX* creat_multiple_matrix(_IN INTEGER rows, _IN INTEGER columns, _IN INTEGER count, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// 创建零矩阵 返回矩阵指针
MATRIX* creat_zero_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// 创建单位矩阵 返回矩阵指针
MATRIX* creat_eye_matrix(_IN INTEGER n, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// 创建对角矩阵 返回矩阵指针
MATRIX* creat_diag_matrix(_IN REAL *X, _IN INDEX L, _OUT ERROR_ID* errorID, _OUT STACKS* S); 
// 矩阵转变为均匀随机矩阵
ERROR_ID matrix_UniformRandom(_IN MATRIX* A);
// 矩阵转变为正态随机矩阵
ERROR_ID matrix_NormalRandom(_IN MATRIX* A);
// 矩阵加法 A+B=C
ERROR_ID matrix_add(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// 矩阵减法 A-B=C
ERROR_ID matrix_subtraction(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// 矩阵数乘 a*B=C
ERROR_ID matrix_numbermulti(_IN REAL A, _IN MATRIX* B, _OUT MATRIX* C);
// 矩阵乘法 A*B=C
ERROR_ID matrix_multiplication(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// 向量相乘得矩阵 v1*v2'=M
ERROR_ID matrix_vector2matrix(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// 矩阵横向拼接 C=[A,B] 
ERROR_ID matrix_rowmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// 矩阵纵向拼接 C=[A;B] 
ERROR_ID matrix_columnmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// 矩阵提取 C=A（Rs:Rs+C->row,Cs:Cs+C->column） 
ERROR_ID matrix_extraction(_IN MATRIX* A, _OUT MATRIX* C, _IN INDEX Rs, _IN INDEX Cs);
// 矩阵赋值 C（Rs:Rs+C->row,Cs:Cs+C->column）=A 
ERROR_ID matrix_valuation(_IN MATRIX* A,  _IN MATRIX* C, _IN INDEX Rs, _IN INDEX Cs);
// 矩阵求行列式
ERROR_ID matrix_det(_IN MATRIX* A, _OUT MATRIX* detA);
double matrix_detsubF1(double array[5][5], int n);
double matrix_detsubF2(double array[5][5], int i, int n);
// 矩阵求逆
ERROR_ID matrix_inverse(_IN MATRIX* A, _OUT MATRIX* invA);
// 矩阵转置
ERROR_ID matrix_transpose(_IN MATRIX* A, _OUT MATRIX* transposeA);
// 矩阵的迹
ERROR_ID matrix_trace(_IN MATRIX* A, _OUT REAL* trace);
// 正定矩阵A的Cholesky 分解，输出下三角矩阵 
ERROR_ID Cholesky_decomposition(_IN MATRIX* A, _OUT MATRIX* L);
// n行n列矩阵A的LUP分解 PA=L*U n行n列下三角L矩阵 n行n列上三角U矩阵 n行n列置换矩阵P
ERROR_ID lup_decomposition(_IN MATRIX* A, _OUT MATRIX* L, _OUT MATRIX* U, _OUT MATRIX* P);
// LUP分解解矩阵方程AX=B，其中A(n*n)，B(n*m)，X(n*m)待求矩阵（写到矩阵B）
ERROR_ID solve_matrix_equation_by_lup_decomposition(_IN MATRIX* A, _IN_OUT MATRIX* B);

#endif
