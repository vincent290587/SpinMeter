/*
 * UDMatrix.cpp
 *
 *  Created on: 28 mrt. 2019
 *      Author: v.golle
 */

#include "UDMatrix.h"
#include "segger_wrapper.h"

UD1D::UD1D(unsigned size): m_size(size) {
}

UDVector::UDVector(unsigned size) : UD1D(size) {
}

UDColumn::UDColumn(unsigned size) : UD1D(size) {
}

UDMatrix::UDMatrix(unsigned _rowSize, unsigned _colSize) : m_rowSize(_rowSize), m_colSize(_colSize) {

	// assign zeros
	vector<udm_type_t> tmp (_colSize,0);
	m_data.assign(_rowSize, tmp);

}

UDMatrix UDMatrix::operator +(UDMatrix& s_mat) {

	UDMatrix res(this->m_rowSize, this->m_rowSize);

	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< s_mat.m_colSize; j++) {

			res.m_data[i][j] = this->m_data[i][j] + s_mat.m_data[i][j];

		}
	}

	return res;
}

UDMatrix UDMatrix::operator -(UDMatrix& s_mat) {

	UDMatrix res(this->m_rowSize, this->m_rowSize);

	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< s_mat.m_colSize; j++) {

			res.m_data[i][j] = this->m_data[i][j] - s_mat.m_data[i][j];

		}
	}

	return res;
}

UDMatrix UDMatrix::operator *(UDMatrix& s_mat) {

	UDMatrix res(this->m_rowSize, s_mat.m_colSize);

	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< s_mat.m_colSize; j++) {
			for (unsigned k=0; k< this->m_colSize; k++) {

				res.m_data[i][j] = this->m_data[i][k] * s_mat.m_data[k][j];

			}
		}
	}

	return res;
}

UDMatrix UDMatrix::transpose() {

	UDMatrix res(this->m_colSize, this->m_rowSize);

	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< this->m_colSize; j++) {

			res.m_data[j][i] = this->m_data[i][j];

		}
	}

	return res;
}

void UDMatrix::print(void) {

	LOG_RAW_INFO("----------  Matrix ---------------");
	LOG_RAW_INFO("\r\n");
	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< this->m_colSize; j++) {

			LOG_RAW_INFO("%f ", this->m_data[i][j] );

		}
		LOG_RAW_INFO("\r\n");
	}


}

/**
 * https://www.geeksforgeeks.org/finding-inverse-of-a-matrix-using-gauss-jordan-method/
 */
UDMatrix UDMatrix::invert() {

	float temp;
	const unsigned order = this->m_rowSize;
	UDMatrix res(this->m_rowSize, this->m_rowSize);

	UDMatrix t_mat(this->m_rowSize, 2*this->m_rowSize);

	// Create the augmented matrix
	// Add the identity matrix
	// of order at the end of orignal matrix.
	for (unsigned i = 0; i < order; i++) {

		for (unsigned j = 0; j < 2 * order; j++) {

			t_mat.m_data[i][j] = 0;

			if (j < order) t_mat.m_data[i][j] = this->m_data[i][j];

			// Add '1' at the diagonal places of
			// the matrix to create a identity matirx
			if (j == (i + order))
				t_mat.m_data[i][j] = 1;
		}
	}

	// Interchange the row of matrix,
	// interchanging of row will start from the last row
	for (unsigned i = order - 1; i > 0; i--) {

		if (t_mat.m_data[i - 1][0] < t_mat.m_data[i][0])
			for (unsigned j = 0; j < 2 * order; j++) {

				// Swapping of the row, if above
				// condition satisfied.
				temp = t_mat.m_data[i][j];
				t_mat.m_data[i][j] = t_mat.m_data[i - 1][j];
				t_mat.m_data[i - 1][j] = temp;
			}
	}

	// Replace a row by sum of itself and a
	// constant multiple of another row of the matrix
	for (unsigned i = 0; i < order; i++) {

		for (unsigned j = 0; j < order; j++) {

			if (j != i) {

				temp = t_mat.m_data[j][i] / t_mat.m_data[i][i];
				for (unsigned k = 0; k < 2 * order; k++) {

					t_mat.m_data[j][k] -= t_mat.m_data[i][k] * temp;
				}
			}
		}
	}

	// Multiply each row by a nonzero integer.
	// Divide row element by the diagonal element
	for (unsigned i = 0; i < order; i++) {

		temp = t_mat.m_data[i][i];
		for (unsigned j = 0; j < order; j++) {

			res.m_data[i][j] = t_mat.m_data[i][j+order] / temp;
		}
	}

	return res;
}

void UDMatrix::unity(void) {

	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< this->m_colSize; j++) {

			this->m_data[i][j] = i == j ? 1:0;

		}
	}

}

void UDMatrix::zeros(void) {

	for (unsigned i=0; i< this->m_rowSize; i++) {
		for (unsigned j=0; j< this->m_colSize; j++) {

			this->m_data[i][j] = 0;

		}
	}

}


