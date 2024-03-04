# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Models the kinematics of a differential drive robot

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1. Take a Vector2D by reference and return a new Vector2D object.
      2. Calculate and return a new Vector2D object representing the unit vector.
      3. Modify the current Vector2D object to represent the unit vector.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

      1. 
         * Pro: Avoids copying the object, reusable.
         * Con: Due to using reference, rather than object, it is less intuitive.
      2. 
         * Pro: Readable, clear code.
         * Con: May accidentally create extra/unnecessary objects (and thereby impact computational performance).
      3.
         * Pro: Efficient, avoids object copies. Relatively less verbose.
         * Con: Modifies original object, which could be a problem if other parts of code rely on it.


   - Which of the methods would you implement and why?

   * I would implement (c), where the Vector2D object is modified to represent the unit vector. It is efficient, as it avoids creating a new object for the normalized vector. The code is also relatively simple, concise, and easy to understand. The most important factor for me was simplicity, as I am not worried about performance gains, and this implementation allowed me to be sure of the function's performance.

2. What is the difference between a class and a struct in C++?

* In classes, members (data and functions) are private by default and therefore can only be accessed from within the class (or by its friends). Members of structs, on the other hand, are public be default. Classes and structs also have different common use cases: classes are used for complex objects with internal states and behaviors, while structs are used for simple data structures with public members and no inheritance.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

* Vector2D is a struct because it represents a simple data structure where the individual members can vary independently (C.2).

* Transform2D is a class because there are non-public members (C.8).

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

* Some of the constructors in Transform2D are explicit to avoid unintended conversions (i.e. from Transform2D to Vector2D) (C.46).


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

* Transform2D::inv() is declared const because it doesn't modify the object's state, while Transform2D::operator*= is not because it does modify the object's state. The C++ Core Guidelines prefers making objects immutable so as to "avoid accidental or hard-to-notice change of value." By default, member functions should be const to give a "more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities."