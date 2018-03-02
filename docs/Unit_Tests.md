 # Unit Testing for RIP

Here's a quick, FAQ style rundown of how to get started with unit testing for RIP, for someone who has never heard of it.
### TL;DR
1. read the [introduction to Google Test unit testing](https://github.com/google/googletest/blob/master/googletest/README.md).
2. review other unit testing code in RIP
3. start by setting up test cases, outline what you want to test
4. Tests should NEVER interface with hardware or make changes that are persistent after the tests.
5. The code being tested should not be changed for testing purposes. Use mock or a fake and virtual/override modifiers sparingly to get access to what you need.
6. A fake or mock (not the same) is a class that overrides parts of the code to intercept and replace behavior with your own. [More about mocks](https://github.com/google/googletest/blob/master/googlemock/README.md)


## Google Test
Before you do anything else, read the [introduction to Google Test unit testing](https://github.com/google/googletest/blob/master/googletest/README.md).

This explains what unit testing is, why it's important, and shows how to actually do it. This, the [advanced primer](https://github.com/google/googletest/blob/master/googletest/docs/AdvancedGuide.md), and other unit tests inside RIP (I would suggest looking at ArduinoGen's unit testing), are the primary resources you'll use to write your unit tests.

 Code gets complicated, and unit tests often reveal issues that are either spotted when the tests are completed, or appear unexpectedly as merges, minor changes, etc. manage to silently break parts of the code. This process of code just breaking, despite seemingly not being touched at all, happens far too often.

 ## Unit Tests: Pre-writing stage
 The goal is to have your tests reach every line of code. We use lcov code coverage to check how much of the code is being reached by the tests. By the time unit testing is complete, every line of code should run as a part of the unit tests.

 Personally, the way I started writing unit tests is to look at the inputs and outputs of each function and thinking of every way I could break it. Anything conceivably possible is worth testing.

 An easy way to come up with test cases is to review any exception handling inside the project you are working on as a hint for scenarios to look and test for. If there isn't sufficient exception handling, ADD IT. Making new exceptions inside RIP is easy. Inside your exceptions.hpp, just add:

 ```c++
 /**
  * [NEW_EX description]
  * @param {[type]} descriptive [description]
  */
 NEW_EX(<descriptive exception name>)
 ```

 Note that a semicolon is not neccesary.

 In general, an easy way for me to categorize tests was to make a

## Basic Example test case
```c++
//returns a double that is a / b (not integer divide)
double SomeClass::divide(int a, int b)
{
    if(b == 0)
    {
        throw DivideByZero("I'm a tutorial");
    }
    else
    {
        return a / b;
    }
    dogCow.moof();   
}
```
This function sucks, it won't work as intended. But at least it has exception handling.
```c++
TEST(ExampleCategory, exampleSpecific)
{
    //see below for annotated descriptions, looks nicer
    //than a bunch of comments.
    std::unique_ptr<SomeFakeClass> test(new SomeFakeClass);

    EXPECT_THROW(test->divide(1, 0), DivideByZero);
    ASSERT_THROW(test->divide(1, 0), DivideByZero);

    EXPECT_DOUBLE_EQ(test->divide(3, 2), 1.5);
    ASSERT_TRUE(test->returnTrue());

    FAIL();
}
```
This test isn't really a super example, but introduces some of the keywords and what a test would look like.

`TEST(ExampleCategory, exampleSpecific)`

The names don't matter, as long as they are descriptive and consistent.


`std::unique_ptr<SomeFakeClass> test(new SomeFakeClass);`

Note that you shouldn't be making actual objects of what you are testing. Use a fake or mock class object. Make sure that everything inside the test body is a pointer, to meet coding standards. Use smart pointers so that they de-allocate after the test.

`EXPECT` means non-fatal, causes the test to fail, `ASSERT` means that execution stops.

Refer to the primer to see all the flavors of assert & expect.

## Mocks/fakes, Encapsulation
[Documentation machine broke, come back later.](https://github.com/google/googletest/blob/master/googlemock/docs/ForDummies.md)

### Example test, using a fake interface
Documentation machine broke, come back later.

## Random Stuff
##### *I wrote something, how do I run the tests?*
Assuming you've successfully built rip [(see the main readme)](../README.md)
the executable should be inside `rip/build/<path to your project>`.
