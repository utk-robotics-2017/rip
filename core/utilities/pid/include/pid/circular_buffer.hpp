#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

namespace rip
{
    namespace pid
    {
        template<typename T>
        class CircularBuffer
        {
        public:
            explicit CircularBuffer(size_t size)
                : m_data(size, 0)
            {}

            /**
             * Returns the number of elements in the buffer
             */
            size_t size() const
            {
                return m_length;
            }

            /**
             * Returns the value at the front of the buffer
             */
            T& front()
            {
                return (*this)[0];
            }

            /**
             * Returns the value at the front of the buffer
             */
            const T& front() const
            {
                return (*this)[0];
            }

            /**
             * Returns the value at the back of the buffer
             */
            T& back()
            {
                if (m_length == 0)
                {
                    return 0;
                }

                return m_data[(m_front + m_length - 1) % m_data.size()];
            }

            /**
             * Returns the value at the back of the buffer
             */
            const T& back() const
            {
                if (m_length == 0)
                {
                    return 0;
                }

                return m_data[(m_front + m_length - 1) % m_data.size()];
            }

            /**
             * Push a new value onto the front of the buffer. The value at
             * the back is overwritten if the buffer is full.
             */
            void push_front(const T& value)
            {
                if (m_data.size() == 0)
                {
                    return;
                }

                m_front = moduloDec(m_front);

                m_data[m_front] = value;

                if (m_length < m_data.size())
                {
                    m_length ++;
                }
            }

            /**
             * Push a new value on the back of the buffer. The value at
             * the front is overwritten if the buffer is full.
             */
            void push_back(const T& value)
            {
                if (m_data.size() == 0)
                {
                    return;
                }

                m_data[(m_front + m_length) % m_data.size()] = value;

                if (m_length < m_data.size())
                {
                    m_length ++;
                }
                else
                {
                    m_front = moduloInc(m_front);
                }
            }

            /**
             * Pop the value at the front of the buffer
             */
            T pop_front()
            {
                // If there are no elements in the buffer, do nothing
                if (m_length == 0)
                {
                    return 0;
                }

                T& temp = m_data[m_front];
                m_front = moduloInc(m_front);
                m_length--;
                return temp;
            }

            /**
             * Pop the value at the back of the buffer
             */
            T pop_back()
            {
                // If there are no elements in the buffer, do nothing
                if (m_length == 0)
                {
                    return 0;
                }

                m_length--;
                return m_data[(m_front + m_length) % m_data.size()];
            }

            void resize(size_t size)
            {
                if (size > m_data.size())
                {
                    // Find end of buffer
                    size_t insert_location = (m_front + m_length) % m_data.size();

                    // If insertion location precedes front of buffer, push front index back
                    if (insert_location <= m_front)
                    {
                        m_front += size - m_data.size();
                    }

                    // Add elements to end of buffer
                    m_data.insert(m_data.begin() + insert_location, size - m_data.size(), 0);
                }
                else if (size < m_data.size())
                {
                    /* 1) Shift element block start at "front" left as many blocks as were
                     *    removed up to but not exceeding buffer[0]
                     * 2) Shrink buffer, which will remove even more elements automatically if
                     *    necessary
                     */
                    size_t elems_to_remove = m_data.size() - size;
                    auto front_iter = m_data.begin() + m_front;
                    if (m_front < elems_to_remove)
                    {
                        /* Remove elements from end of buffer before shifting start of element
                         * block. Doing so saves a few copies.
                         */
                        m_data.erase(front_iter + size, m_data.end());

                        // Shift start of element block to left
                        m_data.erase(m_data.begin(), front_iter);

                        // Update metadata
                        m_front = 0;
                    }
                    else
                    {
                        // Shift start of element block to left
                        m_data.erase(front_iter - elems_to_remove, front_iter);

                        // Update metadata
                        m_front -= elems_to_remove;
                    }

                    /* Length only changes during a shrink if all unused spaces have been
                     * removed. Length decreases as used spaces are removed to meet the
                     * required size.
                     */
                    if (m_length > size)
                    {
                        m_length = size;
                    }
                }
            }

            void reset()
            {
                std::fill(m_data.begin(), m_data.end(), 0);
                m_front = 0;
                m_length = 0;
            }

            T& operator[](size_t index)
            {
                return m_data[(m_front + index) % m_data.size()];
            }

            const T& operator[](size_t index) const
            {
                return m_data[(m_front + index) % m_data.size()];
            }

        private:
            std::vector<T> m_data;

            size_t m_front = 0;
            size_t m_length = 0;

            size_t moduloInc(size_t index)
            {
                return (index + 1) % m_data.size();
            }

            size_t moduloDec(size_t index)
            {
                if (index == 0)
                {
                    return m_data.size() - 1;
                }
                else
                {
                    return index - 1;
                }
            }
        };
    }
}

#endif // CIRCULAR_BUFFER_HPP
