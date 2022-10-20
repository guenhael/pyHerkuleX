======================================
|pyHerkuleX| ``Mirror``
======================================


|release-shield| |GPL3-shield| |pipeline-shield|


pyHerkuleX is a Python package for communicating with
smart HerkuleX servo motors  manufactured by Hyulim Robot company.

To obtain permission to use this code for commercial purposes,
contact Guenhael LE QUILLIEC (mailto:contact@guenhael.com).


Contribute
======================================

`Github <https://github.com/guenhael/pyherkulex>`_ is a **mirror** of the
`official pyHerkuleX repository <https://framagit.org/guenhael/pyherkulex>`_.
This is a publish-only repository and all pull requests are ignored.
Please visit the
`official repository on Framagit <https://framagit.org/guenhael/pyherkulex>`_
to report issues or contribute fixes
(no registration needed, you can also sign in with your GitHub account).


Documentation
======================================

An HTML version of the documentation is available online at:

|documentation-ico| `<https://guenhael.frama.io/pyherkulex/>`_

.. _getting-started-1:

Requirements
======================================

- `Python <https://python.org/>`_ 2.7 or higher, including Python 3.4 and higher
- `pySerial <https://github.com/pyserial/pyserial>`_ 2.6 or higher


.. _getting-started-2:

Download and installation
======================================

Download a copy of the latest public release (current version: |release|):

.. include:: images/images.RST

|zip-ico| `pyherkulex-master.zip <https://framagit.org/guenhael/pyherkulex/-/archive/master/pyherkulex-master.zip>`_

Unzip and run ``python setup.py build install`` from the source
directory to install pyherkulex library on your machine.
Or simply copy-past pyherkulex folder in the working directory of your project.


.. _getting-started-3:

Simple usage example
======================================

This simple example blinks LED in blue, in 1 second interval,
of all HerkuleX servos connected to default serial port
with default baudrate 115200.

.. code-block:: python

  #!/usr/bin/env python

  """
  Blinks LED in blue of all connected HerkuleX servos.
  """

  import pyherkulex as hx
  import time

  broadcast_srv = hx.Servo()
  broadcast_srv.reboot()

  while True:
      broadcast_srv.led = hx.LED_BLUE
      time.sleep(1)
      broadcast_srv.led = hx.LED_OFF
      time.sleep(1)

.. tip::
    * Before running this test, make sure each servos has its own ID,
      different from any other connected servo to avoid any communication error,
      or simply run this test with only one single servo connected to the serial bus.
    * Also make sure no servo has a specific baudrate value
      different than the default (factory) value 115200.
    * Finally, check servo status in case of LED blinking red (alarm).


.. ===================================================================
   =========================== END README ============================
   ===================================================================

.. |release| replace:: 1.1.0


.. |release-shield| image:: https://img.shields.io/badge/Release-v1.1.0-yellow.svg
   :alt: Release 1.1.0
   :target: #


.. |GPL3-shield| image:: https://img.shields.io/badge/License-GPL%20v3-blue.svg
   :alt: License: GPL v3
   :target: https://www.gnu.org/licenses/gpl-3.0

.. |pyHerkuleX| image:: doc_html/_static/git_images/pyherkulex_logo_small.png
   :alt: pyHerkuleX logo

.. |documentation-ico| image:: doc_html/_static/git_images/documentation.png
   :alt: online documentation
   :target: https://guenhael.frama.io/pyherkulex/

.. |zip-ico| image:: doc_html/_static/git_images/zip.png
   :alt: download project archive
   :target: https://framagit.org/guenhael/pyherkulex/-/archive/master/pyherkulex-master.zip

.. |pipeline-shield| image:: https://travis-ci.org/guenhael/pyherkulex.svg?branch=master
   :alt: pipeline status
   :target: https://framagit.org/guenhael/pyherkulex/-/commits/master