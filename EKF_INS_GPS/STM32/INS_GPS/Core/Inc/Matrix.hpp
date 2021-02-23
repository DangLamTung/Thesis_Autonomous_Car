/*
 * Matrix.hpp
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */


/*
 * Matrix.h
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */
#include <stdlib.h>
#include <string.h>




class Matrix {

public:
	int row;
	int col;
	double data[49];
	bool inv = true;
	Matrix(int row, int col);
	virtual ~Matrix();

    int get_value(double* data){
    	for (int i = 0; i < row; i++)
    	    for (int j = 0; j < col; j++)
    	    	this->data[i*col + j] = data[i*col + j];
    	return 0;
    }

//    int print(){
////    	char buffer[20];
//    	std::cout << std::fixed;
//    	for (int i = 0; i < row; i++){
//    	    	    for (int j = 0; j < col; j++){
//    	    	    	cout << data[i*col + j] <<" ";
//    	    	    }
//    	            cout << "\n";
//    	}
//    	cout << "\n";
//    	return 0;
//    }
    double det();
};

Matrix::Matrix(int row, int col){
	    this->row = row;
	    this->col = col;


	    for (int i = 0; i < row; i++)
	    	   for (int j = 0; j < col; j++){
	    		   this->data[i*col + j] = 0;
	    	   }
//	    			  this->data[i*col + j] = 0;
	}

Matrix::~Matrix() {
	// TODO Auto-generated destructor stub

}





Matrix diag_mat(int row, int col) {

	// TODO Auto-generated constructor stub
	Matrix m = Matrix(row,col);
	for (int i = 0; i < row; i++)
		for (int j = 0; j < col; j++)
           if(i==j)
			 m.data[i*col + j] = 1;
   return m;
}
Matrix getCofactor(Matrix A, int p, int q)
{
    Matrix temp = Matrix(A.row - 1,A.col - 1);
    uint8_t i,j,h,k;
    h = 0; k = 0;
    for(i = 0; i<A.row ; i++){
        for(j = 0; j<A.col;j++){
            if(p != i && q != j){
                temp.data[h*temp.col + k++] = A.data[i * A.col + j];
                // cout<< A.data[i * A.col + j] << " ";
                    if (j == A.col - 1)
                {
                    k = 0;
                    h++;
                }
            }

        }
        // cout<< "\n";
    }
    return temp;
}
Matrix mul_mat(Matrix mat1, Matrix mat2) {
   Matrix temp = Matrix(mat1.row, mat2.col);
    if(mat2.row != mat1.col){

    }
    else{
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	       for(int k = 0; k < mat1.col ; k++)
	    	   temp.data[i*temp.col +j] += mat1.data[i*mat1.col + k] * mat2.data[k*mat2.col + j];
    }
   return temp;
}
Matrix add_mat(Matrix mat1, Matrix mat2) {
   Matrix temp = Matrix(mat1.row, mat2.col);
    if(mat2.row != mat1.row){

    }
    else{
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	    	   temp.data[i*temp.col +j] = mat1.data[i*mat1.col + j] + mat2.data[i*mat2.col + j];
    }
   return temp;
}
Matrix sub_mat(Matrix mat1, Matrix mat2) {
   Matrix temp = Matrix(mat1.row, mat2.col);
    if(mat2.row != mat1.row){

    }
    else{
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	    	   temp.data[i*temp.col +j] = mat1.data[i*mat1.col + j] - mat2.data[i*mat2.col + j];
    }
   return temp;
}

Matrix transpose(Matrix A) {
    Matrix L = Matrix(A.col, A.row);

    for (int i = 0; i < A.row; i++)
        for (int j = 0; j < A.col; j++) {
                L.data[j * L.col+ i] = A.data[i * A.col + j];
        }

    return L;
}
double determinant(Matrix mat)
{
    double D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (mat.row == 1)
        return mat.data[0];

    Matrix L = Matrix(mat.col, mat.row); // To store cofactors

    double sign = 1.0;  // To store sign multiplier

     // Iterate for each element of first row
    for (int f = 0; f < mat.col; f++)
    {
        // Getting Cofactor of mat[0][f]
        L =  getCofactor(mat, 0, f);
        D += sign * mat.data[f] * determinant(L);
        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}
Matrix adjoint(Matrix a)
{
    Matrix adj = Matrix(a.row,a.col);
    if (a.row == 1)
    {
        adj.data[0] = 1;

    }

    // temp is used to store cofactors of A[][]
    int sign = 1;


    for (int i=0; i<a.row; i++)
    {
        for (int j=0; j<a.row; j++)
        {
            // Get cofactor of A[i][j]
            Matrix temp = getCofactor(a,i, j);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj.data[j * a.col + i] = (sign)*(determinant(temp));
        }
    }
    return adj;
}

// Function to calculate and store inverse, returns false if
// matrix is singular


Matrix inverse(Matrix a)
{
    Matrix inverse_mat = Matrix(a.col,a.col);
    // Find determinant of A[][]
    double det = determinant(a);
    if (det == 0)
    {
//        cout << "Singular matrix, can't find its inverse";
        return inverse_mat;
    }

    // Find adjoint
    Matrix adj = Matrix(a.col,a.col);
    adj = adjoint(a);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<a.col; i++)
        for (int j=0; j<a.col; j++)
        	inverse_mat.data[i*a.col + j] = adj.data[i*a.col + j]/det;

//    return true;
    return inverse_mat;
}

