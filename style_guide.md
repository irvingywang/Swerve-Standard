# C Style Guide

## Naming Conventions

- **Variables**: `snake_case` (e.g., `total_sum`)
- **Functions**: `snake_case` (e.g., `calculate_area`)
- **Constants**: `ALL_CAPS` (e.g., `MAX_VALUE`)
- **Macros**: `ALL_CAPS` with `snake_case` for multiple words (e.g., `#define MAX_BUFFER_SIZE 1024`)
- **Structs**: `Upper_Case_t` for type names (e.g., `typedef struct Person_t`)
- **Enums**: `Upper_Case_e` for type name, `ALL_CAPS` for values (e.g., `enum Color_e { RED, GREEN, BLUE }`)
- **Boolean Variable**: for state booleans, use all caps IS_BOOL_STATE (e.g., `bool IS_FIRING_ENABLED`)

## Spacing Guidelines

- Use **4 spaces** for indentation, no tabs.
- Place **1 space** around operators (`+`, `-`, `=`, etc.)
- Leave **1 blank line** between function definitions.
- Opening curly braces `{` on the **same line** as the function or statement.

## Comments

- Use `//` for single-line comments.
- Use `/* */` for multi-line comments.

## Examples

```c
#include <stdio.h>

// Macros
#define MAX_BUFFER_SIZE 1024
#define SQUARE(x) ((x) * (x))

// Enum example
typedef enum {
    RED,
    GREEN,
    BLUE
} Color_e;

// Struct example
typedef struct {
    char name[50];
    int age;
    enum Color_e favorite_color;
} Person_Info_t;

// Function prototypes
void print_person_info(Person_Info_t p);
int add_numbers(int a, int b);
int multiply_numbers(int a, int b);

// Main function
int main() {
    Person_Info_t john = {"John Doe", 30, GREEN};
    print_person_info(john);

    int sum = add_numbers(5, 10);
    printf("Sum: %d\n", sum);

    int product = multiply_numbers(4, 6);
    printf("Product: %d\n", product);

    printf("Square of 4: %d\n", SQUARE(4));

    return 0;
} // end main()

// Function implementations
void print_person_info(Person_Info_t p) {
    printf("Name: %s\n", p.name);
    printf("Age: %d\n", p.age);

    printf("Favorite Color: ");
    switch (p.favorite_color) {
        case RED:
            printf("Red\n");
            break;
        case GREEN:
            printf("Green\n");
            break;
        case BLUE:
            printf("Blue\n");
            break;
        default:
            printf("Unknown\n");
    }
} // end print_person_info

// Adds two numbers
int add_numbers(int a, int b) {
    return a + b;
} // end add_numbers

// Multiplies two numbers
int multiply_numbers(int a, int b) {
    return a * b;
} // end multiply_numbers
```
