# Template for Blocks

## Base Class `Block`

The `Block` class serves as the base class for all blocks in the Controller layer. It defines a single pure virtual function `tick()` that derived classes must implement.

```cpp
/**
 * @brief Base interface for all elements of the Controller layer in SIGMA architecture.
 *
 * The Block class serves as the base class for all controller elements.
 * It defines a pure virtual function tick(), which is the main module callback
 * and must be implemented by derived classes.
 */
class Block {
 public:
  /**
   * @brief Pure virtual function of Block callback.
   */
  virtual void tick() = 0;
};
```

## Derivative Classes

Any class derived from the `Block` class must implement the `Constructor` and the single public method `tick()`.

During the initialization of an instance through the `constructor`, references to `Data Channels` are passed to the class. These channels contain the INPUT data and parameters for functioning (optional).

Internal input and output variables are created using the macros `BLOCK_INPUT(type, name)` and `BLOCK_OUTPUT(type, name)`.
<details>
  <summary>Macros code</summary>

  ```cpp
  /**
 * @brief Macro to define an input channel in a Block.
 * This macro creates a protected const reference to an input variable.
 * @param type The data type of the input.
 * @param name The name of the input variable.
 */
#define BLOCK_INPUT(type, name) \
  protected:                     \
  const type& name##_;

/**
 * @brief Macro to define an output channel in a Block.
 * This macro creates a protected reference to an output variable.
 * @param type The data type of the output.
 * @param name The name of the output variable.
 */
#define BLOCK_OUTPUT(type, name) \
  protected:                      \
  type name##_;                   \
                                  \
  public:                         \
  const type& get_##name() const { return name##_; }
  ```
</details>

These macros create variables by adding `_` at the end. For example:
- `BLOCK_INPUT(int32_t, input)` will create a protected const reference `input_` which can be used in the implementation of the `tick()` method.
- `BLOCK_OUTPUT(int32_t, output)` will create a variable `output_` which can be used in the implementation of the `tick()` method. It will also create a public const `get_output()` method that returns reference to the `output_` variable.
> It is important that all internal variables have the postfix `_`.

Local variables used for calculations should be private or protected.

All computation implementation should be in the `tick()` method. The `tick()` method must be overridden with the specific calculations of the class's functionality and specified output. Each call to this method will read the `Data Channels` specified during initialization and update the output values.

Thus, each derived class contains:
- `public`:
    - `Constructor`
    - `tick()`
    - `get_[var name]()` const reference `to` Output Data Channels` (created automatically by the macro)
- `protected`:
    - `const link` to `Input Data Channels` (created automatically by the macro)
    - `output variables`
- `private` or `protected`:
    - `local variables`

### Example of Usage

The `RotateCoordinates` class rotates the coordinates based on input sine and cosine values and stores the resultant sine and cosine values in the output variables.

```cpp
/**
 * @file block_rotate_coordinates.h
 * @brief Header file for the Coordinate Rotation
 *
 * This file contains the definition of the RotateCoordinates class, which
 * implements a method to rotate a set of coordinates based on given sine and cosine values.
 *
 * The rotation operation helps in transforming the coordinates to a new frame
 * of reference, which is useful in various applications such as robotics,
 * computer graphics, and aerospace engineering.
 */

#ifndef BLOCK_ROTATE_COORDINATES_H
#define BLOCK_ROTATE_COORDINATES_H

#include "generic_block.h"

/**
 * @brief Rotate Coordinates
 * This class implements the rotation of coordinates based on input sine and cosine values.
 * @param tick Update the rotation calculations.
 * @param output_sin Resultant sine value after rotation.
 * @param output_cos Resultant cosine value after rotation.
 */
class RotateCoordinates : public Block {
  BLOCK_INPUT(float, input_sin);     // Input sine value of the initial angle
  BLOCK_INPUT(float, input_cos);     // Input cosine value of the initial angle
  BLOCK_INPUT(float, rotation_sin);  // Sine value of the rotation angle
  BLOCK_INPUT(float, rotation_cos);  // Cosine value of the rotation angle
  BLOCK_OUTPUT(float, output_sin);   // Resultant sine value after rotation
  BLOCK_OUTPUT(float, output_cos);   // Resultant cosine value after rotation

 public:
  /**
   * @brief RotateCoordinates constructor.
   * @param input_sin The input sine value of the initial angle.
   * @param input_cos The input cosine value of the initial angle.
   * @param rotation_sin The sine value of the rotation angle.
   * @param rotation_cos The cosine value of the rotation angle.
   */
  RotateCoordinates(const float& input_sin,
                    const float& input_cos,
                    const float& rotation_sin,
                    const float& rotation_cos)
      : input_sin_(input_sin),
        input_cos_(input_cos),
        rotation_sin_(rotation_sin),
        rotation_cos_(rotation_cos) {}

  /**
   * @brief Updates the rotation calculations.
   * This method computes the new sine and cosine values after applying the rotation.
   */
  void tick() override;
};

void RotateCoordinates::tick() {
  // Compute the resultant sine value after rotation
  output_sin_ = input_sin_ * rotation_cos_ + input_cos_ * rotation_sin_;
  
  // Compute the resultant cosine value after rotation
  output_cos_ = input_cos_ * rotation_cos_ - input_sin_ * rotation_sin_;
}

#endif  // BLOCK_ROTATE_COORDINATES_H
```

#### Example of Chaining Classes

Here is an example of how to use the `RotateCoordinates` class in combination with another `RotateCoordinates` where the public variables of one class are used as input to another.

```cpp
#include "block_rotate_coordinates.h"

// Global variables for the integrator input and output
float input_sin = 0.0f, input_cos = 1.0f, rotation_sin = 0.707f, rotation_cos = 0.707f;

// Create instances of the classes
RotateCoordinates rotate1(input_sin, input_cos, rotation_sin, rotation_cos);
RotateCoordinates rotate2(rotate1.get_output_sin(), rotate1.get_output_cos(), rotation_sin, rotation_cos);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Rotation Example");

    // Perform rotation 45 degrees
  rotate1.tick();

  // Perform another rotation 45 degrees
  rotate2.tick();

  // Print results
  Serial.print("Sin: ");
  Serial.println(rotate2.get_output_sin());
  Serial.print("Cos: ");
  Serial.println(rotate2.get_output_cos());
}

void loop() {}
```

In this example:

- The `rotate1` class computes the rotated 45 degrees sine and cosine values from simple variables.
- The `rotate2` class computes the rotated 45 degrees sine and cosine values taken from the output of `rotate1`.

This shows how to chain blocks together where the output of one block becomes the input to another.