/*
 * UDMatrix.h
 *
 *  Created on: 28 mrt. 2019
 *      Author: v.golle
 */

#ifndef LIBRARIES_KALMAN_UDMATRIX_H_
#define LIBRARIES_KALMAN_UDMATRIX_H_

#include <stdint.h>
#include <vector>

using std::vector;

typedef float udm_type_t;

class UDMatrix;

class UD1D {
public:
	UD1D(unsigned size);

protected:
	unsigned m_size;
	vector<udm_type_t> m_vector;
};

class UDVector : protected UD1D {
public:
	UDVector(unsigned size);

private:

	friend class UDMatrix;
};

class UDColumn : protected UD1D {
public:
	UDColumn(unsigned size);

private:

	friend class UDMatrix;
};

class UDMatrix {
public:
	UDMatrix(unsigned _rowSize, unsigned _colSize);

	void unity(void);
	void zeros(void);
	void print(void);

	UDMatrix operator+(UDMatrix &s_mat);
	UDMatrix operator-(UDMatrix &s_mat);
	UDMatrix operator*(UDMatrix &s_mat);

	UDMatrix transpose();
	UDMatrix invert();

	vector< vector<udm_type_t> > m_data;

protected:
	unsigned m_rowSize;
	unsigned m_colSize;

};



#endif /* LIBRARIES_KALMAN_UDMATRIX_H_ */
