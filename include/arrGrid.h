#ifndef ARRGRID_H
#define ARRGRID_H

typedef struct arrGrid
{
    int xCells, yCells;
    double x_bias, y_bias;
    double mCellResolution;
    vector<int8_t> & GridDataVector;

    arrGrid(int xcells, int ycells, double xbias, double ybias, double mcellResolution, vector<int8_t> & VecGrid):
        xCells(xcells), yCells(ycells), x_bias(xbias), y_bias(ybias), mCellResolution(mcellResolution), GridDataVector(VecGrid)
    {  }

} arrGrid;


#endif
