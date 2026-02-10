# Contributing to PERAL Robot Design

Thank you for your interest in contributing to the PERAL robot project! This document provides guidelines for contributing to this repository.

## Ways to Contribute

There are many ways to contribute to this project:

- 🐛 Report bugs and issues
- 💡 Suggest new features or improvements
- 📝 Improve documentation
- 🔧 Submit hardware design improvements
- 💻 Contribute code (firmware or control software)
- 🧪 Share experimental results
- 📖 Write tutorials or guides

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/PERAL_robot_design.git
   cd PERAL_robot_design
   ```
3. **Create a branch** for your changes:
   ```bash
   git checkout -b feature/your-feature-name
   ```

## Development Guidelines

### Hardware Contributions

When contributing hardware designs:

- **CAD Files**: Include both native format and universal formats (STEP, STL)
- **Documentation**: Update relevant README files
- **BOM Updates**: Update BOM.md if adding/changing components
- **Testing**: If possible, build and test your design
- **Photos**: Include photos or renderings of the design

### Software Contributions

When contributing code:

- **Code Style**: Follow existing code style and conventions
- **Comments**: Add clear comments for complex logic
- **Documentation**: Update relevant README files
- **Testing**: Add tests for new functionality
- **Dependencies**: Minimize new dependencies

### Firmware Guidelines

- Use clear variable and function names
- Add comments for hardware-specific code
- Test on actual hardware if possible
- Document pin configurations
- Include timing requirements

### Control Software Guidelines

- Write modular, reusable code
- Include docstrings for functions/classes
- Add type hints where appropriate (Python)
- Write unit tests
- Update requirements.txt if adding dependencies

### Documentation Contributions

- Use clear, concise language
- Include examples where helpful
- Add images or diagrams when appropriate
- Check for spelling and grammar
- Update table of contents if needed

## Commit Messages

Write clear commit messages:

```
Add motor calibration routine to firmware

- Implement automatic motor speed calibration
- Add calibration data storage to EEPROM
- Update documentation with calibration procedure
```

Format:
- First line: Brief summary (50 chars or less)
- Blank line
- Detailed description (wrap at 72 chars)

## Pull Request Process

1. **Update documentation** related to your changes
2. **Test your changes** thoroughly
3. **Update CHANGELOG** (if applicable)
4. **Create pull request** with clear description:
   - What changes were made
   - Why the changes were needed
   - How to test the changes
   - Any breaking changes

5. **Respond to feedback** from reviewers
6. **Wait for approval** before merging

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Hardware design
- [ ] Performance improvement

## Testing
How were the changes tested?

## Checklist
- [ ] Code follows project style
- [ ] Documentation updated
- [ ] Tests added/updated
- [ ] All tests pass
- [ ] BOM updated (if hardware change)
```

## Code Review Process

All contributions go through code review:

1. Maintainers review your pull request
2. Feedback may be provided for improvements
3. Make requested changes and update PR
4. Once approved, changes will be merged

## Reporting Issues

When reporting issues, include:

- **Clear title** describing the problem
- **Description** of what happened vs. expected behavior
- **Steps to reproduce** the issue
- **Environment details** (hardware, software versions)
- **Logs or error messages** if applicable
- **Photos or screenshots** if helpful

### Issue Template

```markdown
## Description
Clear description of the issue

## Steps to Reproduce
1. Step one
2. Step two
3. ...

## Expected Behavior
What should happen

## Actual Behavior
What actually happens

## Environment
- Hardware version: 
- Firmware version:
- Software version:
- OS:

## Additional Context
Any other relevant information
```

## Feature Requests

For feature requests, describe:

- **What** you want to add/change
- **Why** it would be useful
- **How** it might be implemented (if you have ideas)
- **Alternatives** you've considered

## Design Discussions

For major changes:

1. **Open an issue** to discuss the design first
2. Get feedback from maintainers
3. Refine the approach based on discussion
4. Implement after consensus is reached

## Testing

### Hardware Testing
- Build and test physical prototypes
- Verify mechanical fit and function
- Test under expected operating conditions
- Document test results

### Software Testing
- Write unit tests for new functions
- Test integration with existing code
- Verify on actual hardware
- Check edge cases and error handling

### Documentation Testing
- Verify instructions work as written
- Have someone else follow the steps
- Check for clarity and completeness

## Code of Conduct

### Our Standards

- Be respectful and inclusive
- Welcome newcomers
- Accept constructive criticism
- Focus on what's best for the project
- Show empathy towards others

### Unacceptable Behavior

- Harassment or discriminatory language
- Trolling or insulting comments
- Publishing others' private information
- Other unprofessional conduct

## Questions?

If you have questions about contributing:

- Open an issue with the "question" label
- Check existing documentation
- Reach out to maintainers

## Recognition

Contributors will be:

- Listed in project documentation
- Credited in publications (where appropriate)
- Acknowledged in release notes

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file).

## Thank You!

Your contributions make this project better for everyone. We appreciate your time and effort! 🎉

