.. |_nbsp| unicode:: 0xA0 0xA0 0xA0 0xA0
    :trim:

Material specification
======================

Non-Visual Materials
--------------------

Non-visual materials determine how non-visual sensors interact with surfaces.
These materials enable the computation of reflection and transmission coefficients
based on surfaces' physical and spectral properties.

Non-visual materials are parametrized by the following USD attributes:

- ``inputs:nonvisual:base`` — the base material.
- ``inputs:nonvisual:coating`` — optional surface coating.
- ``inputs:nonvisual:attributes`` — optional surface attributes.

These fields are encoded into a single unsigned 16-bit integer (``uint16``) material ID:
the lower byte holds the base material index, the lower 3 bits of the upper byte encode the coating,
and the upper 5 bits encode the attributes as a bitfield.

.. code-block:: text

    attributes  coatings    base material
    xxxxx       xxx         xxxxxxxx

Supported values
^^^^^^^^^^^^^^^^

The following tables details all the supported base materials, coatings, and attributes.

.. note::

    When parametrizing non-visual materials, the base material field is required,
    while the coatings and attributes fields are optional.

.. csv-table:: Base
    :file: ../data/specifications/base.csv
    :header-rows: 1

.. csv-table:: Coating
    :file: ../data/specifications/coating.csv
    :header-rows: 1

.. csv-table:: Attribute
    :file: ../data/specifications/attribute.csv
    :header-rows: 1
