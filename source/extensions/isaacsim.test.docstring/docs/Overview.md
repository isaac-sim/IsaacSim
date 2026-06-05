# Overview

The isaacsim.test.docstring extension provides test case base classes for validating Python docstring examples using Python's doctest module. It enables developers to automatically test code examples within docstrings to ensure they execute correctly and produce expected outputs, supporting both standalone unittest environments and Kit's async testing framework.

## Key Components

### {class}`StandaloneDocTestCase <isaacsim.test.docstring.StandaloneDocTestCase>`

**{class}`StandaloneDocTestCase <isaacsim.test.docstring.StandaloneDocTestCase>`** serves as the base class for standalone test cases that validate docstring examples outside of Kit environments. This class extends Python's unittest.TestCase and provides specialized methods for docstring testing in traditional Python testing scenarios.

The class offers two primary testing methods:
- `assertDocTest()` - Tests docstring examples for a single class, module member, or function
- `assertDocTests()` - Tests docstring examples for all members within a module or class

```python
import unittest
from isaacsim.test.docstring import StandaloneDocTestCase
import my_module

class TestDocstrings(StandaloneDocTestCase):
    def test_my_module_docstrings(self):
        # Test all members of a module
        self.assertDocTests(my_module)

    def test_single_function(self):
        # Test a specific function
        self.assertDocTest(my_module.some_function)
```

### {class}`AsyncDocTestCase <isaacsim.test.docstring.AsyncDocTestCase>`

**{class}`AsyncDocTestCase <isaacsim.test.docstring.AsyncDocTestCase>`** extends **omni.kit.test.AsyncTestCase** to provide docstring testing capabilities within Kit's async testing environment. This class is specifically designed for testing extension modules and async code examples that require Kit's application context.

The async version includes an additional `await_update` parameter that allows Kit application updates between docstring tests, ensuring proper execution timing in async environments.

```python
from isaacsim.test.docstring import AsyncDocTestCase
import my_extension_module

class TestDocstrings(AsyncDocTestCase):
    async def test_my_module_docstrings(self):
        await self.assertDocTests(my_extension_module)

    async def test_with_options(self):
        await self.assertDocTests(
            my_extension_module.MyClass,
            exclude=[my_extension_module.MyClass.internal_method],
            await_update=True
        )
```

## Functionality

### Doctest Directive Support

Both test case classes support standard doctest directives that control test execution behavior:

- `# doctest: +NO_CHECK` - Runs the example but skips output verification
- `# doctest: +SKIP` - Completely skips the example during testing
- `# doctest: +ELLIPSIS` - Allows `...` in expected output to match any substring
- `# doctest: +NORMALIZE_WHITESPACE` - Ignores whitespace differences in output comparison

### Test Customization

The extension provides several options for customizing docstring test execution:

- **Exclusion filtering** - Skip specific members from testing using the `exclude` parameter
- **Execution ordering** - Control the order of test execution with the `order` parameter
- **Failure handling** - Use `stop_on_failure` to halt testing at the first encountered failure
- **Custom messaging** - Provide custom assertion messages for failed tests

### Integration

Uses **omni.kit.test** to provide async testing capabilities within Kit environments, allowing docstring tests to properly interact with Kit's application lifecycle and async execution model.
