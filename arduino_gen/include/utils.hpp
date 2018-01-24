#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <vector>
#include <map>

namespace rip
{
    namespace arduinogen
    {
        template <typename T>
        struct identity { typedef T type; };

        template<typename K, typename V>
        std::vector<V> mmap_to_vector(std::multimap<K, V> mmap, typename identity<K>::type key)
        {
            std::vector<V> values;

            std::transform(mmap.lower_bound(key),
                           mmap.upper_bound(key),
                           std::back_inserter(values),
                           [](std::pair<K, V> element)
            {
                return element.second;
            });

            return values;
        }

        void replace(std::string& base, const std::string& replacee, const std::string& replacer);
    }
}

#endif // UTILS_HPP
