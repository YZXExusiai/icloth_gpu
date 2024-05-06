#pragma once
inline size_t find_index(int i, const std::vector<int>& indices) {
    for (size_t ii = 0; ii < indices.size(); ii++)
        if (indices[ii] == i)
            return ii;
    return indices.size();
}
template <typename T> inline int find(const T& x, const std::vector<T>& xs) {
    for (int i = 0; i < xs.size(); i++) if (xs[i] == x) return i; return -1;
}
template <typename T> struct SpVec {
    std::vector<int> indices;
    std::vector<T> entries;
    T operator[] (int i) const {
        size_t j = find_index(i, indices);
        if (j >= indices.size() || indices[j] != i)
            return T(0);
        else
            return entries[j];
    }
    T& operator[] (int i) {
        size_t j = find_index(i, indices);
        if (j >= indices.size() || indices[j] != i)
            insert_index((int)i, (int)j, indices, entries);
        return entries[j];
    }
};
template <typename T> struct SpMat {
    int m, n;
    std::vector< SpVec<T> > rows;
    SpMat() : m(0), n(0), rows() {}
    explicit SpMat(int m, int n) : m(m), n(n), rows(m) {}
    T operator() (int i, int j) const {
        return rows[i][j];
    }
    T& operator() (int i, int j) {
        return rows[i][j];
    }
};