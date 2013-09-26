/**
 * A class for online mean calculation
 * Algorithm from: Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
 * operator* for non scalar T shold be the point wise operation
 */
template <class T>
class onlineMean {
    T current_mean;
    unsigned int sample_num;
    T M2;
    
    public:
    onlineMean();
    ~onlineMean() {};
    void feedSample(const T& sample);
    void reset();
    T getMean();
    T getVariance();
    unsigned int getSampleNum();
};


template<class T>
onlineMean<T>::onlineMean() {
    this->reset();
}

template<class T>
void onlineMean<T>::feedSample(const T& sample) {
    T delta;
    sample_num++;
    if( sample_num == 1 ) {
        current_mean = sample;
    } else {
        delta = sample-current_mean;
        current_mean += delta/sample_num;
        if( sample == 2 ) {
            M2 = delta*(sample-current_mean);
        } else {
            M2 = M2 + delta*(sample - current_mean);
        }
    }
    return;
}

template<class T>
void onlineMean<T>::reset() {
    sample_num = 0;
}

template<class T>
T onlineMean<T>::getMean() {
    return current_mean;
}

template<class T>
T onlineMean<T>::getVariance() {
    return M2/(sample_num-1);
}

template<class T>
unsigned int  onlineMean<T>::getSampleNum() {
    return sample_num;
}

