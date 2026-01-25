/* 矩阵运算库源文件 */

#include "matrix.h"


//初始化堆栈 
VOID init_stack(_IN_OUT STACKS* S)
{
    srand(time(NULL));
    if (S == NULL)
    {
        return;
    }
    memset(S,0,sizeof(STACKS));
}

//释放堆栈 
VOID free_stack(_IN STACKS* S)
{
    MATRIX_NODE* matrixNode = NULL;
    MATRIX_ELEMENT_NODE* matrixElementNode = NULL;
    if (S == NULL)
    {
        return;
    }
    while (S->matrixNode != NULL)
    {
        matrixNode = S->matrixNode;
        S->matrixNode = matrixNode->next;
        free(matrixNode->ptr);
        matrixNode->ptr = NULL;
        free(matrixNode);
        matrixNode = NULL;
    }
    while (S->matrixElementNode != NULL)
    {
        matrixElementNode = S->matrixElementNode;
        S->matrixElementNode = matrixElementNode->next;
        free(matrixElementNode->ptr);
        matrixElementNode->ptr = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;
    }
    
}


// 打印输出矩阵
VOID print_matrix(MATRIX* a, STRING string)
{
    INDEX i,j;
    printf("matrix %s:",string);
    printf("\n");
    for (i = 0; i < a->rows; i++)
    {
        for (j = 0; j < a->columns; j++)
        {
            printf("%f  ", a->p[i * a->columns + j]);
        }
        printf("\n");
    }
    printf("\n");
}

// 创建矩阵
MATRIX* creat_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    MATRIX* matrix = NULL;
    MATRIX_NODE* matrixNode = NULL;
    MATRIX_ELEMENT_NODE* matrixElementNode = NULL;

    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (rows <= 0 || columns <= 0 || S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return NULL;
    }

    matrix = (MATRIX*)malloc(sizeof(MATRIX));
    matrixNode = (MATRIX_NODE*)malloc(sizeof(MATRIX_NODE));
    matrixElementNode = (MATRIX_ELEMENT_NODE*)malloc(sizeof(MATRIX_ELEMENT_NODE));
    if (matrix == NULL || matrixNode == NULL || matrixElementNode == NULL)
    {
        free(matrix);
        matrix = NULL;
        free(matrixNode);
        matrixNode = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;

        *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        return NULL;
    }

    matrix->rows = rows;
    matrix->columns = columns;
    matrix->p = (REAL*)malloc(rows * columns * sizeof(REAL));  //确保matrix非空才能执行指针操作
    if (matrix->p == NULL)
    {
        free(matrix->p);
        matrix->p = NULL;
        free(matrix);
        matrix = NULL;
        free(matrixNode);
        matrixNode = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;

        *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        return NULL;
    }
    matrixNode->ptr = matrix;
    matrixNode->next = S->matrixNode;
    S->matrixNode = matrixNode;

    matrixElementNode->ptr = matrix->p;
    matrixElementNode->next = S->matrixElementNode;
    S->matrixElementNode = matrixElementNode;

    return matrix;
}

// 创建多个矩阵
MATRIX* creat_multiple_matrix(_IN INTEGER rows, _IN INTEGER columns, _IN INTEGER count, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    INDEX i;
    MATRIX* matrix = NULL, *p = NULL;
    MATRIX_NODE* matrixNode = NULL;

    if (errorID ==NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (rows <= 0 || columns <= 0 || count <= 0 || S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return NULL;
    }

    matrix = (MATRIX*)malloc(count * sizeof(MATRIX));
    matrixNode = (MATRIX_NODE*)malloc(sizeof(MATRIX_NODE));
    if (matrix == NULL || matrixNode == NULL)
    {
        free(matrix);
        matrix = NULL;
        free(matrixNode);
        matrixNode = NULL;

        *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        return NULL;
    }

    for (i = 0; i < count; i++)
    {
        p = creat_matrix(rows, columns, errorID, S);
        if (p == NULL)
        {
            free(matrix);
            matrix = NULL;
            free(matrixNode);
            matrixNode = NULL;

            *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
            return NULL;
        }
        matrix[i] = *p;
    }

    matrixNode->ptr = matrix;
    matrixNode->next = S->matrixNode;
    S->matrixNode = matrixNode;

    return matrix;
}

// 创建零矩阵
MATRIX* creat_zero_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    MATRIX* matrix = NULL;
    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (rows <= 0 || columns <= 0 || S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return NULL;
    }

    matrix = creat_matrix(rows, columns, errorID, S);
    if (*errorID == _ERROR_NO_ERROR)
    {
        memset(matrix->p, 0, rows * columns * sizeof(REAL)); //赋值
    }

    return matrix;
}

// 创建单位矩阵
MATRIX* creat_eye_matrix(_IN INTEGER n, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    INDEX i;
    MATRIX* matrix = NULL;
    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (n <= 0|| S == NULL)
    {f
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return NULL;
    }

    matrix = creat_matrix(n, n, errorID, S);
    if (*errorID == _ERROR_NO_ERROR)
    {
        memset(matrix->p, 0, n * n * sizeof(REAL));  //先创建零矩阵
        for (i = 0; i < n; i++)
        {
            matrix->p[i * n + i] = 1.0;  //后主对角线赋值1.0
        }
    }
    return matrix;
}

// 创建对角矩阵 返回矩阵指针
MATRIX* creat_diag_matrix(_IN REAL *X, _IN INDEX L, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    INDEX i;
    MATRIX* matrix = NULL;
    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return NULL;
    }

    matrix = creat_matrix(L, L, errorID, S);
    if (*errorID == _ERROR_NO_ERROR)
    {
        memset(matrix->p, 0, L * L * sizeof(REAL));  //先创建零矩阵
        for (i = 0; i < L; i++)
        {
            matrix->p[i * L + i] = X[i];  //后主对角线赋值1.0
        }
    }
    return matrix;
}

// 矩阵转变为0-1均匀随机矩阵
ERROR_ID matrix_UniformRandom(_IN MATRIX* A)
{
    INDEX i;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    if (errorID == _ERROR_NO_ERROR)
    {
        for (i = 0; i < (A->rows*A->columns) ; i++)
        {
            A->p[i] = (REAL)rand()/RAND_MAX;  //后主对角线赋值1.0
        }
        
    }

    return errorID;
}

// 矩阵转变为单位正态随机矩阵
ERROR_ID matrix_NormalRandom(_IN MATRIX* A)

{
    INDEX i;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    REAL n1,n2;
    if (errorID == _ERROR_NO_ERROR)
    {
        for (i = 0; i < (A->rows*A->columns) ; i++)
        {
            n1 = (REAL)rand()/RAND_MAX;  
            n2 = (REAL)rand()/RAND_MAX;  
            A->p[i] = sqrt(-2*log(n1))*cos(n2*2*PI);  //后主对角线赋值1.0
        }
        
    }
    
    return errorID;
} 

// 矩阵加法 A+B=C
ERROR_ID matrix_add(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j;
    INTEGER rows, columns;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != B->rows || A->rows != C->rows || B->rows != C->rows || A->columns != B->columns || A->columns != C->columns || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }

    rows = A->rows;
    columns = A->columns;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < columns; j++)
        {
            C->p[i * columns + j] = A->p[i * columns + j] + B->p[i * columns + j];
        }
    }
    return errorID;
}

// 矩阵减法 A-B=C
ERROR_ID matrix_subtraction(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j;
    INTEGER rows, columns;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != B->rows || A->rows != C->rows || B->rows != C->rows || A->columns != B->columns || A->columns != C->columns || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }

    rows = A->rows;
    columns = A->columns;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < columns; j++)
        {
            C->p[i * columns + j] = A->p[i * columns + j] - B->p[i * columns + j];
        }
    }
    return errorID;
}

// 矩阵数乘 C=a*B
ERROR_ID matrix_numbermulti(_IN REAL A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (B->rows != C->rows || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_MULTIPLICATION;
        return errorID;
    }

    for (i = 0; i < B->rows; i++)
    {
        for (j = 0; j < B->columns; j++)
        {
            C->p[i * B->columns + j] = A * B->p[i * B->columns + j];
        }
    }
    return errorID;
}

// 矩阵乘法 C=A*B
ERROR_ID matrix_multiplication(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j,k;
    REAL sum;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->columns != B->rows || A->rows != C->rows || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_MULTIPLICATION;
        return errorID;
    }

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < B->columns; j++)
        {
            sum = 0.0;
            for (k = 0; k < A->columns; k++)
            {
                sum += A->p[i * A->columns + k] * B->p[k * B->columns + j];
            }
            C->p[i * B->columns + j] = sum;
        }
    }
    return errorID;
}

// 向量相乘得矩阵 v1*v2'=M
ERROR_ID matrix_vector2matrix(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != C->rows || B->columns != C->columns || A->columns != 1 || B->rows != 1)
    {
        errorID = _ERROR_MATRIX_MULTIPLICATION;
        return errorID;
    }

    for (i = 0; i < C->rows; i++)
    {
        for (j = 0; j < C->columns; j++)
        {
            C->p[i * C->columns + j] += A->p[i] * B->p[j];
        }
    }
    return errorID;
} 

// 矩阵横向拼接 C=[A,B] 
ERROR_ID matrix_rowmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != B->rows || A->rows != C->rows || A->columns + B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }
    
    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[i * A->columns + j];
        }
        for (j = 0; j < B->columns; j++)
        {
            C->p[i * C->columns + j + A->columns] = B->p[i * B->columns + j];
        }
    }
    return errorID;
}

// 矩阵纵向拼接 C=[A;B] 
ERROR_ID matrix_columnmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->columns != B->columns || A->columns != C->columns || A->rows + B->rows != C->rows)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }
    
    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[i * A->columns + j];
        }
    }
    for (i = 0; i < B->rows; i++)
    {
        for (j = 0; j < B->columns; j++)
        {
            C->p[(i + A->columns) * C->columns + j] = B->p[i * B->columns + j];
        }
    }
    return errorID;
}

// 矩阵提取 C=A（Rs:Rs+C->row,Cs:Cs+C->column）
ERROR_ID matrix_extraction(_IN MATRIX* A, _OUT MATRIX* C, _IN INDEX Rs, _IN INDEX Cs)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows < C->rows + Rs || A->columns < C->columns + Cs)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }
    
    for (i = 0; i < C->rows; i++)
    {
        for (j = 0; j < C->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[(i + Rs)* A->columns + j + Cs];
        }
    }
    return errorID;
}
// 矩阵赋值 C（Rs:Rs+C->row,Cs:Cs+C->column）=A 
ERROR_ID matrix_valuation(_IN MATRIX* A,  _IN MATRIX* C, _IN INDEX Rs, _IN INDEX Cs)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows + Rs > C->rows || A->columns + Cs > C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }
    
    for (i = Rs; i < Rs + A->rows; i++)
    {
        for (j = Cs; j < Cs + A->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[(i - Rs)* A->columns + j -+ Cs];
        }
    }
    return errorID;
}
// 矩阵求行列式
ERROR_ID matrix_det(_IN MATRIX* A, _OUT MATRIX* detA)
{
	int i,j;
	double array[5][5];
    ERROR_ID errorID = _ERROR_NO_ERROR;
	
    if (A == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }
    
    if (A->rows != A->columns || detA->rows != 1 || detA->columns != 1 )
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        return errorID;
    }
	for(i=0;i<A->rows;i++)
	{
		for(j=0;j<A->rows;j++)
		{
			array[i][j] = A->p[j+i*A->columns];
		}
	}
	detA->p[0]=matrix_detsubF1(array,A->rows);
	return errorID;
}
double matrix_detsubF1(double array[5][5], int n)
{
	int i;
	double M, sum = 0.0;
	if (n == 1)//一阶行列式直接得出结果
		return array[0][0];
	else if (n > 1)
	{
		for (i = 0; i < n; i++)//按第一行展开
		{
			M = matrix_detsubF2(array, i, n);
			sum += pow(-1, i + 2) * array[0][i] * M;
		}
	}
	return sum;
}
double matrix_detsubF2(double array[5][5], int i, int n)
{	
	int j,k;
	double SubArray[5][5];
	
	//构造余子式的过程
		for (j = 0; j < n - 1; j++)
		{
			for (k = 0; k < n - 1; k++)
			{
				if (k < i)
					SubArray[j][k] = array[j + 1][k];
				else if (k >= i)
					SubArray[j][k] = array[j + 1][k + 1];
			}
		}
		
	return matrix_detsubF1(SubArray, n - 1);//计算余子式的行列式
}
// 矩阵求逆
ERROR_ID matrix_inverse(_IN MATRIX* A, _OUT MATRIX* invA)
{
    INDEX i;
    INTEGER n;
    MATRIX* Atemp = NULL;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    STACKS S;

    if (A == NULL || invA == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        return errorID;
    }

    init_stack(&S);

    n = A->rows;
    Atemp = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR)
    {
        goto EXIT;
    }

    memcpy(Atemp->p, A->p, n * n * sizeof(REAL));
    memset(invA->p, 0, n * n * sizeof(REAL));
    for (i = 0; i < n; i++)
    {
        invA->p[i * n + i] = 1.0; //这一步使得invA为单位阵
    }

    errorID = solve_matrix_equation_by_lup_decomposition(Atemp, invA);

EXIT:
    free_stack(&S);
    return errorID;
}

// 矩阵转置
ERROR_ID matrix_transpose(_IN MATRIX* A, _OUT MATRIX* transposeA)
{
    INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || transposeA == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != transposeA->columns || A->columns != transposeA->rows)
    {
        errorID = _ERROR_MATRIX_TRANSPOSE_FAILED;
        return errorID;
    }

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->columns; j++)
        {
            transposeA->p[j * A->rows + i] = A->p[i * A->columns + j];
        }
    }
    return errorID;
}

// 矩阵的迹
ERROR_ID matrix_trace(_IN MATRIX* A, _OUT REAL* trace)
{
    INDEX i;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || trace == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        return errorID;
    }

    *trace = 0.0;
    for (i = 0; i < A->rows; i++)
    {
        *trace += A->p[i * A->columns + i];
    }
    return errorID;
}

// 正定矩阵A的Cholesky 分解  LT*L=R
ERROR_ID Cholesky_decomposition(_IN MATRIX* A, _OUT MATRIX* L)
{
    INDEX i, j, k, n;
    double sum = 0;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || L == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
    }
    n = A->rows;
    memcpy(L->p, A->p, n * n * sizeof(REAL));
    for(k = 0; k < n; k++)
    {
        sum = 0;
        for(i = 0; i < k; i++)
            sum += L->p[k * n + i] * L->p[k * n + i];
        sum = A->p[k * n + k] - sum;
        L->p[k * n + k] = sqrt(sum > 0 ? sum : 0);
        for(i = k + 1; i < n; i++)
        {
            sum = 0;
            for(j = 0; j < k; j++)
                sum += L->p[i * n + j] * L->p[k * n + j];
            L->p[i * n + k] = (A->p[i * n + k] - sum) / L->p[k * n + k];
        }
        for(j = 0; j < k; j++)
            L->p[j * n + k] = 0;
    }
    return errorID;
}

// n行n列矩阵A的LUP分解 PA=L*U n行n列下三角L矩阵 n行n列上三角U矩阵 n行n列置换矩阵P
ERROR_ID lup_decomposition(_IN MATRIX* A, _OUT MATRIX* L, _OUT MATRIX* U, _OUT MATRIX* P)
{
    INDEX i, j, k, index, s, t;
    INTEGER n;
    REAL maxvalue, temp;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || L == NULL || U == NULL || P == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        return errorID;
    }

    n = A->rows;
    memcpy(U->p, A->p, n * n * sizeof(REAL));
    memset(L->p, 0, n * n * sizeof(REAL));
    memset(P->p, 0, n * n * sizeof(REAL));
    for (i = 0; i < n; i++)
    {
        L->p[i * n + i] = 1.0;
        P->p[i * n + i] = 1.0;
    }
    for (j = 0; j < n - 1; j++)
    {
        // Select i(>=j) that maxmizes |U(i,j)|
        index = -1;
        maxvalue = 0.0;
        for (i = j; i < n; i++)
        {
            temp = fabs(U->p[i * n + j]);
            if (temp > maxvalue)
            {
                maxvalue = temp;
                index = i;
            }
        }
        if (index == -1)
        {
            continue;
        }

        for (k = j; k < n; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = U->p[s];
            U->p[s] = U->p[t];
            U->p[t] = temp;
        }

        for (k = 0; k < j; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = L->p[s];
            L->p[s] = L->p[t];
            L->p[t] = temp;
        }

        for (k = 0; k < n; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = P->p[s];
            P->p[s] = P->p[t];
            P->p[t] = temp;
        }

        for (i = j + 1; i < n; i++)
        {
            s = i * n + j;
            L->p[s] = U->p[s] / U->p[j * n + j];
            for (k = j; k < n; k++)
            {
                U->p[i * n + k] -= L->p[s] * U->p[j * n + k];
            }
        }
    }
    return errorID;
}
// LUP分解解矩阵方程AX=B，其中A(n*n)，B(n*m)，X(n*m)待求矩阵（写到矩阵B）
ERROR_ID solve_matrix_equation_by_lup_decomposition(_IN MATRIX* A, _IN_OUT MATRIX* B)
{
    INDEX i, j, k, index, s, t;
    INTEGER n, m;
    REAL sum, maxvalue, temp;
    MATRIX* L = NULL, *U = NULL, *y = NULL;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    STACKS S;

    if (A == NULL || B == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        return errorID;
    }

    init_stack(&S);

    n = A->rows;
    m = B->columns;

    L = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR) goto EXIT;

    U = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR) goto EXIT;

    y = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR) goto EXIT;

    memcpy(U->p, A->p, n * n * sizeof(REAL));
    memset(L->p, 0, n * n * sizeof(REAL));
    for (i = 0; i < n; i++)
    {
        L->p[i * n + i] = 1.0;
    }

    for (j = 0; j < n - 1; j++)
    {
        // Select i(>=j) that maxmizes |U(i,j)|
        index = -1;
        maxvalue = 0.0;
        for (i = j; i < n; i++)
        {
            temp = fabs(U->p[i * n + j]);
            if (temp > maxvalue)
            {
                maxvalue = temp;
                index = i;
            }
        }
        if (index == -1)
        {
            continue;
        }

        for (k = j; k < n; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = U->p[s];
            U->p[s] = U->p[t];
            U->p[t] = temp;
        }

        for (k = 0; k < j; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = L->p[s];
            L->p[s] = L->p[t];
            L->p[t] = temp;
        }

        for (k = 0; k < m; k++)
        {
            s = j * m + k;
            t = index * m + k;
            temp = B->p[s];
            B->p[s] = B->p[t];
            B->p[t] = temp;
        }

        for (i = j + 1; i < n; i++)
        {
            s = i * n + j;
            L->p[s] = U->p[s] / U->p[j * n + j];
            for (k = j; k < n; k++)
            {
                U->p[i * n + k] -= L->p[s] * U->p[j * n + k];
            }
        }
    }

    for (i = 0; i < n; i++)
    {
        if (fabs(U->p[i * n + i]) < 1.0e-20)
        {
            errorID = _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS;
            goto EXIT;
        }
    }

    // L*y=C
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            sum = 0.0;
            for (k = 0; k < i; k++)
            {
                sum += L->p[i * n + k] * y->p[k * m + j];
            }
            y->p[i * m + j] = B->p[i * m + j] - sum;
        }
    }

    // U*x=y
    for (j = 0; j < m; j++)
    {
        for (i = n - 1; i >= 0; i--)
        {
            sum = 0.0;
            for (k = i + 1; k < n; k++)
            {
                sum += U->p[i * n + k] * B->p[k * m + j];
            }
            B->p[i * m + j] = (y->p[i * m + j] - sum) / U->p[i * n + i];
        }
    }
EXIT:
    free_stack(&S);
    return errorID;
}

ERROR_ID EigenValueVecter(_IN MATRIX* A, _OUT MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j,n;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    
	n = A->rows;
	    if (A->rows != B->rows || B->rows != C->rows || C->columns != 1)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        return errorID;
    }

    if (A->rows != A->columns || B->rows != B->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        return errorID;
    }
    
	float **array = (float **)malloc(n * sizeof(float *));
	if (array == NULL)
	{
		printf("error :申请数组内存空间失败\n");
		return _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
	}
	for (i = 0; i < n; i++)
	{
		array[i] = (float *)malloc(n * sizeof(float));
		if (array[i] == NULL)
		{
			printf("error :申请数组子内存空间失败\n");
			return _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
		}
	}
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			array[i][j] = A->p[i * n + j];
		}
	}
	float *eig = (float *)malloc(n * sizeof(float));
	float **Result = Matrix_Jac_Eig(array, n, eig);
	printf("特征矩阵:\n");
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B->p[i * n + j] = Result[i][j];
			printf("%f ", Result[i][j]);
		}
		printf("\n");
	}
	printf("特征值向量:\n");
	for (i = 0; i < n; i++)
	{
		C->p[i] = eig[i];
		printf("%f \n", eig[i]);
	}
	Matrix_Free(Result, n, n);
	free(eig);
	eig = NULL;
	
    return errorID;;
}

float** Matrix_Jac_Eig(float **array, int n, float *eig)
{
	//先copy一份array在temp_mat中，因为我实在堆区申请的空间,在对其进行处理
	//的过程中会修改原矩阵的值,因此要存储起来,到最后函数返回的
	//时候再重新赋值
	int i, j, flag, k;
	flag = 0;
	k = 0;
	float sum = 0;
	float **temp_mat = (float **)malloc(n * sizeof(float *));
	for (i = 0; i < n; i++)
	{
		temp_mat[i] = (float *)malloc(n * sizeof(float));
	}
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			temp_mat[i][j] = array[i][j];
		}
	}
	//判断是否为对称矩阵
	for (i = 0; i < n; i++)
	{
		for (j = i; j < n; j++)
		{
			if (array[i][j] != array[j][i])
			{
				flag = 1;
				break;
			}
		}
	}
	if (flag == 1)
	{
		printf("error in Matrix_Eig: 输入并非是对称矩阵:\n");
		return NULL;
	}
	else
	{
		//开始执行算法
		int p, q;
		float thresh = 0.0000000001;
		float max = array[0][1];
		float tan_angle, sin_angle, cos_angle;
		float **result = (float **)malloc(n * sizeof(float *));
		if (result == NULL)
		{
			printf("error in Matrix_Eig:申请空间失败\n");
			return NULL;
		}
		float **result_temp = (float **)malloc(n * sizeof(float *));
		if (result_temp == NULL)
		{
			printf("error in Matrix_Eig:申请空间失败\n");
			return NULL;
		}
		float **rot = (float **)malloc(n * sizeof(float *));
		if (rot == NULL)
		{
			printf("error in Matrix_Eig:申请空间失败\n");
			return NULL;
		}
		float **mat = (float **)malloc(n * sizeof(float *));
		if (mat == NULL)
		{
			printf("error in Matrix_Eig:申请空间失败\n");
			return NULL;
		}
		for (i = 0; i < n; i++)
		{
			result[i] = (float *)malloc(n * sizeof(float));
			if (result[i] == NULL)
			{
				printf("error in Matrix_Eig:申请子空间失败\n");
				return NULL;
			}
			result_temp[i] = (float *)malloc(n * sizeof(float));
			if (result_temp[i] == NULL)
			{
				printf("error in Matrix_Eig:申请子空间失败\n");
				return NULL;
			}
			rot[i] = (float *)malloc(n * sizeof(float));
			if (rot[i] == NULL)
			{
				printf("error in Matrix_Eig:申请子空间失败\n");
				return NULL;
			}
			mat[i] = (float *)malloc(n * sizeof(float));
			if (mat[i] == NULL)
			{
				printf("error in Matrix_Eig:申请子空间失败\n");
				return NULL;
			}
		}
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				if (i == j)
				{
					result[i][j] = 1;
				}
				else
				{
					result[i][j] = 0;
				}
			}
		}
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				if (i == j)
				{
					mat[i][j] = 1;
				}
				else
				{
					mat[i][j] = 0;
				}
			}
		}
		max = array[0][1];
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				if (i == j)
				{
					continue;
				}
				else
				{
					if (fabs(array[i][j]) >= fabs(max))
					{
						max = array[i][j];
						p = i;
						q = j;
					}
					else
					{
						continue;
					}
				}
			}
		}
		while (fabs(max) > thresh)
		{
			if (fabs(max) < thresh)
			{
				break;
			}
			tan_angle = -2 * array[p][q] / (array[q][q] - array[p][p]);
			sin_angle = sin(0.5*atan(tan_angle));
			cos_angle = cos(0.5*atan(tan_angle));
			for (i = 0; i < n; i++)
			{
				for (j = 0; j < n; j++)
				{

					if (i == j)
					{
						mat[i][j] = 1;
					}
					else
					{
						mat[i][j] = 0;
					}
				}
			}
			mat[p][p] = cos_angle;
			mat[q][q] = cos_angle;
			mat[q][p] = sin_angle;
			mat[p][q] = -sin_angle;
			for (i = 0; i < n; i++)
			{
				for (j = 0; j < n; j++)
				{
					rot[i][j] = array[i][j];
				}
			}
			for (j = 0; j < n; j++)
			{
				rot[p][j] = cos_angle*array[p][j] + sin_angle*array[q][j];
				rot[q][j] = -sin_angle*array[p][j] + cos_angle*array[q][j];
				rot[j][p] = cos_angle*array[j][p] + sin_angle*array[j][q];
				rot[j][q] = -sin_angle*array[j][p] + cos_angle*array[j][q];
			}
			rot[p][p] = array[p][p] * cos_angle*cos_angle +
				array[q][q] * sin_angle*sin_angle +
				2 * array[p][q] * cos_angle*sin_angle;
			rot[q][q] = array[q][q] * cos_angle*cos_angle +
				array[p][p] * sin_angle*sin_angle -
				2 * array[p][q] * cos_angle*sin_angle;
			rot[p][q] = 0.5f*(array[q][q] - array[p][p]) * 2 * sin_angle*cos_angle +
				array[p][q] * (2 * cos_angle*cos_angle - 1);
			rot[q][p] = 0.5f*(array[q][q] - array[p][p]) * 2 * sin_angle*cos_angle +
				array[p][q] * (2 * cos_angle*cos_angle - 1);
			for (i = 0; i < n; i++)
			{
				for (j = 0; j < n; j++)
				{
					array[i][j] = rot[i][j];
				}
			}
			max = array[0][1];
			for (i = 0; i < n; i++)
			{
				for (j = 0; j < n; j++)
				{
					if (i == j)
					{
						continue;
					}
					else
					{
						if (fabs(array[i][j]) >= fabs(max))
						{
							max = array[i][j];
							p = i;
							q = j;
						}
						else
						{
							continue;
						}
					}
				}
			}
			for (i = 0; i < n; i++)
			{
				eig[i] = array[i][i];
			}
			for (i = 0; i < n; i++)
			{
				for (j = 0; j < n; j++)
				{
					sum = 0;
					for (k = 0; k < n; k++)
					{
						sum = sum + result[i][k] * mat[k][j];
					}
					result_temp[i][j] = sum;
				}
			}
			for (i = 0; i < n; i++)
			{
				for (j = 0; j < n; j++)
				{
					result[i][j] = result_temp[i][j];
				}
			}
		}
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				array[i][j] = temp_mat[i][j];
			}
		}
		Matrix_Free(result_temp, n, n);
		Matrix_Free(rot, n, n);
		Matrix_Free(mat, n, n);
		Matrix_Free(temp_mat, n, n);
		return result;
	}
}
int Matrix_Free(float **tmp, int m, int n)
{
	int i, j;
	if (tmp == NULL)
	{
		return(1);
	}
	for (i = 0; i < m; i++)
	{
		if (tmp[i] != NULL)
		{
			free(tmp[i]);
			tmp[i] = NULL;
		}
	}
	if (tmp != NULL)
	{
		free(tmp);
		tmp = NULL;
	}
	return(0);
}


