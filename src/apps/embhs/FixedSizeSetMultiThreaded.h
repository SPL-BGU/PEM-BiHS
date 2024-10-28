#ifndef FixedSizeSetMultiThreaded_h
#define FixedSizeSetMultiThreaded_h

#include <cstring>
#include <functional>
#include <cstddef>
#include <vector>
#include <mutex>
#include <memory>
#include <cstdio>
#include <atomic>
#include <iostream>

template<typename T>
class fssmtIterator;

template<typename T, class Hash = std::hash<T>>
class FixedSizeSetMultiThreaded {
protected:
    struct field {
        T item;
        field *next;
        bool valid;
    };
    field **hashTable;
    field *memory;
    std::atomic<size_t> currentMemoryEntry;
    size_t capacity;
    size_t maxHashTableSize;
    size_t hashTableSize;
    size_t removed;
    Hash h;
    std::mutex *mutexes;
    std::mutex mem_entry;

public:
    FixedSizeSetMultiThreaded(size_t capacity);

    ~FixedSizeSetMultiThreaded();

    void resize(size_t capacity);

    void swap(FixedSizeSetMultiThreaded<T, Hash> &s);

    void clear();

    size_t size() { return currentMemoryEntry/*-removed*/; }

    typedef fssmtIterator<field> iterator;

    iterator begin() { return iterator(&memory[0]); }

    iterator end() const { return iterator(&memory[currentMemoryEntry]); }

    iterator find(const T &item) const {
        size_t hash = h(item);
        for (field *f = hashTable[hash % hashTableSize]; f; f = f->next) {
            if (f->item == item && f->valid)
                return iterator(f);
        }
        return end();
    }

    void erase(iterator &i);

    void insert(const T &item);

    void PrintStats();

    size_t GetCurrentSize() {
        return capacity * sizeof(field) + hashTableSize * sizeof(field *) + maxHashTableSize * sizeof(std::mutex);
    }
};

template<typename T, class Hash>
void FixedSizeSetMultiThreaded<T, Hash>::swap(FixedSizeSetMultiThreaded<T, Hash> &s) {
    field **a, *b;
    size_t c, d, e, f;
    std::mutex *g;
    uint64_t i;
    a = hashTable;
    b = memory;
    c = currentMemoryEntry;
    d = capacity;
    e = hashTableSize;
    f = removed;
    g = mutexes;
    *this = s;
    s.hashTable = a;
    s.memory = b;
    s.currentMemoryEntry = c;
    s.capacity = d;
    s.hashTableSize = e;
    s.removed = f;
    s.mutexes = g;
}


template<typename T, class Hash>
FixedSizeSetMultiThreaded<T, Hash>::FixedSizeSetMultiThreaded(size_t capacity) {
    maxHashTableSize = 250000000;
    hashTableSize = std::min(maxHashTableSize, (2 * capacity) | 1);
    mutexes = new std::mutex[maxHashTableSize];
    hashTable = new field *[hashTableSize];
    memory = new field[capacity];
    memset(memory, 0x0, capacity * sizeof(field));
    memset(hashTable, 0x0, hashTableSize * sizeof(field *));
    currentMemoryEntry = 0;
    this->capacity = capacity;
    removed = 0;
}

template<typename T, class Hash>
FixedSizeSetMultiThreaded<T, Hash>::~FixedSizeSetMultiThreaded() {
    delete[] mutexes;
    delete[] hashTable;
    delete[] memory;
    memory = 0;
    hashTable = 0;
    currentMemoryEntry = 0;
    capacity = 0;
    removed = 0;
}

template<typename T, class Hash>
void FixedSizeSetMultiThreaded<T, Hash>::resize(size_t capacity) {
    this->capacity = capacity;
    delete[] memory;
    delete[] hashTable;
    hashTableSize = std::min(maxHashTableSize, (2 * capacity) | 1);//capacity*2+3;
    hashTable = new field *[hashTableSize];
    memory = new field[capacity];
    memset(memory, 0x0, capacity * sizeof(field));
    memset(hashTable, 0x0, hashTableSize * sizeof(field *));
    currentMemoryEntry = 0;
    removed = 0;
}

template<typename T, class Hash>
void FixedSizeSetMultiThreaded<T, Hash>::clear() {
    currentMemoryEntry = 0;
    removed = 0;
    memset(hashTable, 0x0, hashTableSize * sizeof(field *));
    memset(memory, 0x0, capacity * sizeof(field));
}

template<typename T, class Hash>
void FixedSizeSetMultiThreaded<T, Hash>::PrintStats() {
    std::vector<int> dist;
    for (int x = 0; x < hashTableSize; x++) {
        int len = 0;
        for (field *f = hashTable[x]; f; f = f->next) {
            len++;
        }
        if (len >= dist.size())
            dist.resize(len + 1);
        dist[len]++;
    }
    for (int x = 0; x < dist.size(); x++)
        printf("%d : %d\n", x, dist[x]);
}

template<typename T, class Hash>
void FixedSizeSetMultiThreaded<T, Hash>::erase(iterator &i) {
//	if ((*i).valid == true)
//	{
    (*i).valid = false;
//		removed++;
//	}
}

template<typename T, class Hash>
void FixedSizeSetMultiThreaded<T, Hash>::insert(const T &item) {
    size_t hash = h(item);
    size_t location = hash % hashTableSize;
    mutexes[location].lock();
    if (hashTable[location] == 0) {
        size_t currentEntry = currentMemoryEntry.fetch_add(1, std::memory_order_relaxed);
        field *f = &memory[currentEntry];
        f->item = item;
        f->next = 0;
        f->valid = true;
        hashTable[location] = f;
    } else {
        //std::cout << "Collision";
        for (field *t = hashTable[location]; t; t = t->next) {
            if (t->item == item)
                break;
            if (t->next == 0) {
                size_t currentEntry = currentMemoryEntry.fetch_add(1, std::memory_order_relaxed);
                field *f = &memory[currentEntry];
                f->item = item;
                f->next = 0;
                f->valid = true;
                t->next = f;
            }
        }
    }
    mutexes[location].unlock();
}


template<typename T>
class fssmtIterator : public std::iterator<std::random_access_iterator_tag, T, ptrdiff_t, T *, T &> {
public:
    fssmtIterator(T *ptr = nullptr) { m_ptr = ptr; }

    fssmtIterator(const fssmtIterator<T> &rawIterator) = default;

    ~fssmtIterator() {}

    fssmtIterator<T> &operator=(const fssmtIterator<T> &rawIterator) = default;

    fssmtIterator<T> &operator=(T *ptr) {
        m_ptr = ptr;
        return (*this);
    }

    operator bool() const {
        if (m_ptr)
            return true;
        else
            return false;
    }

    bool operator==(const fssmtIterator<T> &rawIterator) const { return (m_ptr == rawIterator.getConstPtr()); }

    bool operator!=(const fssmtIterator<T> &rawIterator) const { return (m_ptr != rawIterator.getConstPtr()); }

    fssmtIterator<T> &operator+=(const ptrdiff_t &movement) {
        m_ptr += movement;
        return (*this);
    }

    fssmtIterator<T> &operator-=(const ptrdiff_t &movement) {
        m_ptr -= movement;
        return (*this);
    }

    fssmtIterator<T> &operator++() {
        ++m_ptr;
        return (*this);
    }

    fssmtIterator<T> &operator--() {
        --m_ptr;
        return (*this);
    }

    fssmtIterator<T> operator++(int) {
        auto temp(*this);
        ++m_ptr;
        return temp;
    }

    fssmtIterator<T> operator--(int) {
        auto temp(*this);
        --m_ptr;
        return temp;
    }

    fssmtIterator<T> operator+(const ptrdiff_t &movement) {
        auto oldPtr = m_ptr;
        m_ptr += movement;
        auto temp(*this);
        m_ptr = oldPtr;
        return temp;
    }

    fssmtIterator<T> operator-(const ptrdiff_t &movement) {
        auto oldPtr = m_ptr;
        m_ptr -= movement;
        auto temp(*this);
        m_ptr = oldPtr;
        return temp;
    }

    ptrdiff_t operator-(const fssmtIterator<T> &rawIterator) {
        return std::distance(rawIterator.getPtr(), this->getPtr());
    }

    T &operator*() { return *m_ptr; }

    const T &operator*() const { return *m_ptr; }

    T *operator->() { return m_ptr; }

    T *getPtr() const { return m_ptr; }

    const T *getConstPtr() const { return m_ptr; }

protected:

    T *m_ptr;
};
//-------------------------------------------------------------------

#endif /* FixedSizeSetMultiThreaded_h */
