// Types
typedef float float32;
typedef double float64;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int8_t sint8;
typedef int16_t sint16;
typedef int32_t sint32;

// Subsampling only implemented for 128
enum Sampling_Method_t { STANDARD_SGM = 0, STRIPED_SGM = 1, STRIPED_SGM_SUBSAMPLE_2 = 2, STRIPED_SGM_SUBSAMPLE_4 = 3 };

// 5x5 Semi-Global Matching
template<typename T>
void processCensus5x5SGM(T* leftImg, T* rightImg, float32* output, float32* dispImgRight,
                         int width, int height, int method, uint16 paths, const int numThreads, 
                         const int numStrips, const int dispCount);

// 9x7 HCWS Census measure
template<typename T>
void processCensus9x7SGM(T* leftImg, T* rightImg, float32* output, float32* dispImgRight,
                         int width, int height, int method, uint16 paths, const int numThreads, 
                         const int numStrips, const int dispCount);
