

class Singleton
{
public:
    // Deleted copy constructor and assignment operator to prevent copies
    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;

    static Singleton& Instaance()
    {
        static Singleton instance;
        return instance;
    }
private:
    // Private constructor/destructor ensures that Singleton can only be created/destroyed inside the class
    Singleton()
    {

    }

    ~Singleton()
    {

    }
};