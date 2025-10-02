from typing import Tuple, Any
import importlib


def import_module(import_statement: str) -> Tuple[str, Any]:
    """
    Imports module given a string.

    Args:
        import_statement(str): String to be imported in the following form:
        - "import pkg.mod"
        - "import pkg.mod as alias"
        - "from pkg.mod import member"
        - "from pkg.mod import member as alias"
    Returns:
        (str): The local name the caller should bind to. This is either the alias, the member,
                 or the last module of the import path.
        (Any): The imported module or the imported member.
    Raises:
        ValueError if the string isn't one of the supported forms.
        ModuleNotFoundError / AttributeError if resolution fails.
    """
        
    s = import_statement.strip()

    # import pkg.mod [as alias]
    if s.startswith("import "):
        rest = s[len("import "):].strip()
        parts = rest.split()
        if len(parts) == 1:
            module_path = parts[0]
            alias = module_path.rsplit(".", 1)[-1]
        elif len(parts) == 3 and parts[1] == "as":
            module_path, _, alias = parts
        else:
            raise ValueError("Use 'import pkg.mod [as alias]' (single binding only).")
        if module_path.startswith("."):
            raise ValueError("Relative imports are not supported.")
        obj = importlib.import_module(module_path)
        return alias, obj

    # from pkg.mod import member [as alias]
    elif s.startswith("from "):
        rest = s[len("from "):].strip()
        try:
            module_path, right = rest.split(" import ", 1)
        except ValueError:
            raise ValueError("Use 'from pkg.mod import member [as alias]'.")
        module_path = module_path.strip()
        parts = right.strip().split()
        if len(parts) == 1:
            member, alias = parts[0], parts[0]
        elif len(parts) == 3 and parts[1] == "as":
            member, _, alias = parts
        else:
            raise ValueError("Only a single member import is supported.")
        if module_path.startswith(".") or member == "*":
            raise ValueError("Relative or star imports are not supported.")
        # Try submodule first, then attribute on parent
        try:
            obj = importlib.import_module(f"{module_path}.{member}")
        except ModuleNotFoundError:
            parent = importlib.import_module(module_path)
            obj = getattr(parent, member)
        return alias, obj

    raise ValueError("Statement must start with 'import ' or 'from '.")