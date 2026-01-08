# Code Example Templates

This directory contains templates for creating consistent code examples throughout the Physical AI & Humanoid Robotics Book.

## Template Structure

Each code example should follow the structure outlined in `example_template.py`. This ensures consistency across all examples in the book.

## Using the Template

1. Copy `example_template.py` to your module's code-examples directory
2. Rename the file to something descriptive (e.g., `publisher_example.py`)
3. Update the class name and node name
4. Add your specific implementation
5. Update the docstrings and comments to reflect your implementation

## Template Features

- Proper ROS 2 node initialization
- Standard logging practices
- Exception handling for graceful shutdown
- Clear separation of concerns
- Well-documented code structure
- Consistent naming conventions

## Conventions

- Use descriptive node names
- Follow ROS 2 naming conventions (snake_case for nodes, publishers, subscribers)
- Include proper logging using `self.get_logger()`
- Handle shutdown gracefully with try/finally blocks
- Use appropriate QoS profiles for your use case
- Include type hints where appropriate