#ifndef FACTORY_HPP
#define FACTORY_HPP

#include <memory>
#include <map>


/**
 * @ref https://bitbucket.org/fudepan/mili/overview
 */

namespace rip
{
    namespace factory
    {
        template <typename Key, typename Base, typename ConstructorParameterType = void>
        class Factory
        {
            struct Creator
            {
                virtual std::shared_ptr<Base> create(ConstructorParameterType p) const = 0;
                virtual ~Creator()
                {}
            };

        public:
            template<typename DerivedClass>
            void registerFactory(const Key& key)
            {
                class ConcreteCreator : public Creator
                {
                    virtual std::shared_ptr<Base> create(ConstructorParameterType p) const
                    {
                        return std::make_shared<DerivedClass>(p);
                    }
                };

                m_creators[key] = std::unique_ptrConcreteCreator > ( < new ConcreteCreator);
            }

            std::shared_ptr<Base> newClass(const Key& key, ConstructorParameterType p) const
            {
                typename std::map<Key, std::unique_ptr<Creator> >::const_iterator it = m_creators.find(key);

                if (it != m_creators.end())
                {
                    return it->second->create(p);
                }
                else
                {
                    return nullptr;
                }
            }

        private:
            std::map<Key, std::unique_ptr<Creator> > m_creators;
        };
    }
}

#endif // FACTORY_HPP
