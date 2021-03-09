#ifndef __MATRIX_VECT_PRIVATE_CPP
#define __MATRIX_VECT_PRIVATE_CPP

template <class T> class vect{
	// TODO: make the ints const ints, or better size_types.  Also add checks for success / failure on new[] calls.
public:

	// Constructors and Destructors

	vect(){
		m_Data = new T[10];
		m_nSize = 0;
		m_nCapacity = 10;
	}

	vect(const int n){
		m_Data = new T[n];
		m_nSize = n;
		m_nCapacity = n;
	}

	vect(const int n, const T & t){
		m_Data = new T[n];
		m_nSize = n;
		m_nCapacity = n;

		for(int i = 0; i < m_nSize; i++)
			m_Data[i] = t;
	}

	vect(const vect & ref){
		m_Data = new T[ref.m_nCapacity];
		m_nCapacity = ref.m_nCapacity;
		m_nSize = ref.m_nSize;

		for(int i = 0; i < ref.m_nSize; i++)
			m_Data[i] = ref[i];
	}

	~vect(){
		delete [] m_Data;
	}

	// Operators

	vect & operator=(const vect & rhs){
		if(&rhs != this){
			delete [] m_Data;
			m_Data = new T[rhs.m_nCapacity];
			m_nCapacity = rhs.m_nCapacity;
			m_nSize = rhs.m_nSize;

			for(int i = 0; i < rhs.m_nSize; i++)
				m_Data[i] = rhs[i];
		}
		return *this;
	}

	T & operator[](const int n){
		return m_Data[n];
	}

	const T & operator[](const int n) const{
		return m_Data[n];
	}

	// Public Methods

	int size() const{
		return m_nSize;
	}

	int capacity() const{
		return m_nCapacity;
	}

	bool empty() const{
		return m_nSize == 0;
	}

	void reserve(const int n){
		if(n > m_nCapacity){
			T * tmpData = new T[n];
			for(int i = 0; i < m_nSize; i++)
				tmpData[i] = m_Data[i];
			delete [] m_Data;
			m_Data = tmpData;
			m_nCapacity = n;
		}
	}

	T & front(){
		return m_Data[0];
	}

	const T & front() const{
		return m_Data[0];
	}

	T & back(){
		return m_Data[m_nSize-1];
	}

	const T & back() const{
		return m_Data[m_nSize-1];
	}

	void push_back(const T & ref){
		if(m_nSize == m_nCapacity)
			reserve(m_nSize * 2);
		m_Data[m_nSize++] = ref;
	}

	void pop_back(){
		if(m_nSize > 0)
			m_nSize--;
	}

	void swap(vect & ref){
		T * tmpData;
		int nTmp;

		tmpData = ref.m_Data;
		ref.m_Data = m_Data;
		m_Data = tmpData;

		nTmp = ref.m_nCapacity;
		ref.m_nCapacity = m_nCapacity;
		m_nCapacity = nTmp;

		nTmp = ref.m_nSize;
		ref.m_nSize = m_nSize;
		m_nSize = nTmp;
	}

	void clear(){
		m_nSize = 0;
	}

	void resize(const int n, const T & t = T()){
		if(n > m_nSize){
			reserve(n);
			for(int i = m_nSize; i < n; i++)
				m_Data[i] = t;
		}
		m_nSize = n;
	}

	T * GetData(){
		return m_Data;
	}

private:
	T * m_Data;
	int m_nSize;
	int m_nCapacity;
};

#endif
