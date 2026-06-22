# Public API for module isaacsim.examples.browser:

## Classes

- class ExampleBrowserModel(TreeFolderBrowserModel)
  - def __init__(self, *args: object, **kwargs: object)
  - def set_widget(self, widget: object)
  - def register_example(self, **kwargs: object)
  - def get_category_items(self, item: CollectionItem) -> list[CategoryItem]
  - def get_detail_items(self, item: ExampleCategoryItem) -> list[DetailItem]
  - def execute(self, item: DetailItem)
  - def deregister_example(self, name: str, category: str)
  - def refresh_browser(self)

- class ExampleBrowserWindow(ui.Window)
  - WINDOW_TITLE: str
  - def __init__(self, model: ExampleBrowserModel, visible: bool = True)

## Functions

- def get_instance() -> Optional[ExampleBrowserExtension]
