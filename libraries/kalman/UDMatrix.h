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

class UDMatrix {
public:
	UDMatrix(void);
	UDMatrix(UDMatrix &mat);
	UDMatrix(unsigned _rowSize, unsigned _colSize);

	void set(unsigned x, unsigned y, udm_type_t val);
	void unity(float res = 1.);
	void ones(float res = 1.);
	void zeros(void);
	void print(void);
	void div(float val);
	void mul(float val);
	void resize(unsigned _rowSize, unsigned _colSize);

	UDMatrix operator+(UDMatrix &s_mat);
	UDMatrix operator-(UDMatrix &s_mat);
	UDMatrix operator*(UDMatrix &s_mat);

	UDMatrix transpose();
	UDMatrix invert();

protected:
	unsigned m_rowSize;
	unsigned m_colSize;

	vector< vector<udm_type_t> > m_data;

};



#endif /* LIBRARIES_KALMAN_UDMATRIX_H_ */
