# Public API for module isaacsim.gui.content_browser:

## Classes

- class ExtendedFileInfo(DetailFrameController)
  - MockListEntry: Unknown
  - def __init__(self)
  - def build_header(self, collapsed: bool, title: str)

- class IsaacCollection(CollectionItem)
  - def __init__(self)
  - def create_add_new_item(self) -> Optional[AddNewItem]
  - def create_child_item(self, name: str, path: str, is_folder: bool = True) -> Optional[IsaacConnectionItem]
  - async def populate_children_async(self)
