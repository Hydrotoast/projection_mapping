#ifndef BUFFERS_H
#define BUFFERS_H

#include <mutex>
#include <condition_variable>

namespace buffers {
  // Shared buffers can be read from or written to by swapping the internal
  // buffer pointers. Reading from and writing to the shared buffer is a blocking
  // operation.
  template <typename T=uint16_t>
  class SharedBuffer {
  public:
    SharedBuffer(size_t width, size_t height);
    ~SharedBuffer();

    SharedBuffer(const SharedBuffer& buffer) = delete;
    SharedBuffer& operator=(const SharedBuffer& buffer) = delete;
    SharedBuffer(SharedBuffer&& buffer) = delete;
    SharedBuffer& operator=(SharedBuffer&& buffer) = delete;

    // Accessors

    size_t width() const;
    size_t height() const;
    bool is_ready() const;

    // Blocking IO

    SharedBuffer<T>& write_to(T *&buffer);
    SharedBuffer<T>& read_from(T *&buffer);
  private:
    int width_;
    int height_;

    T *buffer_;
    std::mutex mutex_;
    std::condition_variable ready_condition_;
    bool is_ready_;
  };

  template <typename T>
  SharedBuffer<T>::SharedBuffer(size_t width, size_t height)
    : width_(width)
    , height_(height)
    , buffer_(new T[width * height]) {}

  template <typename T>
  SharedBuffer<T>::~SharedBuffer() {
    delete [] buffer_;
  }

  template <typename T>
  size_t SharedBuffer<T>::width() const { return width_; }

  template <typename T>
  size_t SharedBuffer<T>::height() const { return height_; }

  template <typename T>
  bool SharedBuffer<T>::is_ready() const { return is_ready_; }

  template <typename T>
  SharedBuffer<T>& SharedBuffer<T>::write_to(T *&buffer) {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_condition_.wait(lock, [this] { return this->is_ready_ == true; });

    std::swap(buffer_, buffer);

    is_ready_ = false;
    return *this;
  }

  template <typename T>
  SharedBuffer<T>& SharedBuffer<T>::read_from(T *&buffer) {
    std::lock_guard<std::mutex> guard(mutex_);

    std::swap(buffer_, buffer);

    is_ready_ = true;
    ready_condition_.notify_one();
    return *this;
  }

  template <typename T>
  SharedBuffer<T>& operator>>(SharedBuffer<T>& lhs, T *&rhs) {
    return lhs.write_to(rhs);
  }

  template <typename T>
  SharedBuffer<T>& operator<<(SharedBuffer<T>& lhs, T *&rhs) {
    return lhs.read_from(rhs);
  }
}

#endif
