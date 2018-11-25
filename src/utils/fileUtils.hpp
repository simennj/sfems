#ifndef TMR4160_FILE_UTILS_H
#define TMR4160_FILE_UTILS_H

int getVectorCountFromFile(const char *filename);

void getFloatVectorFromFile(const char *filename, int count, float *array);

void getIntVectorFromFile(const char *filename, int count, int *array);

char * getShaderSource(const char *filename);

int loadConstants(
        const char *filename,
        double *k_p, double *k_i, double *k_d,
        double *targetPosition,
        double *motorCenter, double *motorRadius
);

char *pathAppend(char *result, int resultSize, const char *path1, const char *path2);

#endif //TMR4160_FILE_UTILS_H
