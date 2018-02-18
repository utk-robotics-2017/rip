#ifndef FACTORY_REGISTRY_HPP
#define FACTORY_REGISTRY_HPP

/**
 * @ref https://bitbucket.org/fudepan/mili/overview
 */

namespace rip
{
    namespace factory
    {
        template <typename BaseClass_, typename Key_, typename ConstructorParameterType_>
        struct FactoryTraits
        {
            typedef BaseClass_ BaseClass;
            typedef Key_ Key;
            typedef ConstructorParameterType_ ConstructorParameterType;
        };

        template <typename DerivedRegistry, class Traits>
        class BaseFactoryRegistry
        {
        protected:
            typedef typename Traits::BaseClass BaseClass;
            typedef typename Traits::Key Key;
            typedef typename Traits::ConstructorParameterType ConstructorParameterType;

            unsigned int m_users;
            static std::shared_ptr<DerivedRegistry> m_singleton;
            Factory<Key, BaseClass, ConstructorParameterType> m_fc;

            BaseFactoryRegistry()
                : m_users(0)
            {}

            virtual ~BaseFactoryRegistry()
            {
                deregisterFactory();
            }

            template <typename DerivedClass>
            void _register_factory(const Key& Key)
            {
                ++m_users;
                fc.template registerFactory<DerivedClass>(k);
            }

            bool _deregisterFactory()
            {
                --users;
                return users == 0;
            }

            typename Factory<Key, BaseClass>::KeyIteraor _getConstructibleObjectsKeys()
            {
                return fc.getConstructibleObjectsKeys();
            }
        public:
            template<typename DerivedClass>
            static void registerFactory(const Key& k)
            {
                if (m_singleton == nullptr)
                {
                    m_singleton = std::make_shared<DerivedRegistry>();
                }
                return m_singleton->template _register_factory<DerivedClass>(k);
            }

            static void deregisterFactory()
            {
                if (m_singleton->_deregisterFactory())
                {
                    m_singleton.reset(nullptr);
                }
            }

            static typename Factory<Key, BaseClass>::KeyIteraor getConstructibleObjectsKeys()
            {
                return m_singleton->_getConstructibleObjectsKeys();
            }
        };

        template<typename DerivedRegistry, class Traits>
        std::shared_ptr<DerivedRegistry> BaseFactoryRegistry<DerivedRegistry, Traits>::m_singleton = nullptr;

        template<typename BaseClass, typename Key = std::string, class ConstructorParameterType = void>
        class FactoryRegistry : public BaseFactoryRegistry< FactoryRegistry<BaseClass, Key, ConstructorParameterType>,
            FactoryTraits<BaseClass, Key, ConstructorParameterType> >
        {
        public:
            static std::shared_ptr<BaseClass> newClass(const Key& k, ConstructorParameterType p)
            {
                return BaseFactoryRegistry<FactoryRegistry, FactoryTraits<BaseClass, Key, ConstructorParameterType> >::m_singleton->_newClass(k, p);
            }
        private:
            std::shared_ptr<BaseClass> _newClass(const Key& k, ConstructorParameterType p)
            {
                return this->fc.newClass(k, p);
            }
        };

        template <class BaseClass, class Key>
        class FactoryRegistry<BaseClass, Key, void> : public BaseFactoryRegistry <   FactoryRegistry <BaseClass, Key, void>,
            FactoryTraits   <BaseClass, Key, void> >
        {
        public:
            static std::shared_ptr<BaseClass> newClass(const Key& k)
            {
                return BaseFactoryRegistry<FactoryRegistry, FactoryTraits<BaseClass, Key, void> >::instance->_new_class(k);
            }
        private:
            std::shared_ptr<BaseClass> _newClass(const Key& k)
            {
                return this->fc.newClass(k);
            }
        };

        template<typename BaseClass, typename DerivedClass, typename Key, typename ConstructorParameterType = void>
        class Registerer
        {
        public:
            Registerer(const Key& k)
            {
                factory::FactoryRegistry<BaseClass, Key, ConstructorParameterType>::template registerFactory<DerivedClass>(k);
            }
            ~Registerer()
            {
                factory::FactoryRegistry<BaseClass, Key, ConstructorParameterType>::deregisterFactory();
            }
        };
    }

#define REGISTER_FACTORIZABLE_CLASS(BaseClassName, DerivedClassName, keytype, key)\
    	static factory::Registerer<BaseClassName, DerivedClassName, keytype> r##BaseClassName##DerivedClassName(key)

#define REGISTER_FACTORIZABLE_CLASS_WITH_ARG(BaseClassName, DerivedClassName, keytype, key, ConstructorParameterType)\
    	static factory::Register<BaseClassName, DerivedClassName, keytype, ConstructorParameterType> rr##BaseClassName##DerivedClassName(key)
}

#endif // FACTORY_REGISTRY_HPP
