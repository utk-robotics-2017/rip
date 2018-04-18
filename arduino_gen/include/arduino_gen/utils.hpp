#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <vector>
#include <map>
#include <sstream>

namespace rip
{
    namespace arduinogen
    {
        template<typename K, typename V>
        K select_first(std::pair<K, V> pair)
        {
            return pair.first;
        }

        template<typename K, typename V>
        V select_second(std::pair<K, V> pair)
        {
            return pair.second;
        }

        template <typename T>
        struct identity { typedef T type; };

        template<typename K, typename V>
        std::vector<K> get_map_keys(std::map<K, V> map)
        {
            std::vector<K> keys;
            std::transform(map.begin(), map.end(), std::back_inserter(keys), select_first<K, V>);
            return keys;
        }

        template<typename K, typename V>
        std::vector<K> get_mmap_keys(std::multimap<K, V> mmap)
        {
            std::vector<K> keys;

            // auto -> std::multimap<K, V>::iterator
            for(auto it = mmap.begin(), end = mmap.end();
                it != end; it = mmap.upper_bound(it->first))
            {
                keys.emplace_back(it->first);
            }

            return keys;
        }

        template<typename K, typename V>
        std::vector<V> get_mmap_values_at_index(std::multimap<K, V> mmap, typename identity<K>::type key)
        {
            std::vector<V> values;

            std::transform(mmap.lower_bound(key),
                           mmap.upper_bound(key),
                           std::back_inserter(values),
                           select_second<K, V>);

            return values;
        }

        void replace(std::string& base, const std::string& replacee, const std::string& replacer);

        template<typename T>
        void split(const std::string &s, char delim, T result)
        {
            std::stringstream ss(s);
            std::string item;
            while (std::getline(ss, item, delim))
            {
                if (!item.empty())
                {
                    *(result++) = item;
                }
            }
        }

        std::vector<std::string> split(const std::string &s, char delim);
    }
}

#endif // UTILS_HPP
