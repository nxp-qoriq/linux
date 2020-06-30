.. SPDX-License-Identifier: GPL-2.0

=========================
MDIO bus and PHYs in ACPI
=========================

The PHYs on an mdiobus are probed and registered using
fwnode_mdiobus_register_phy().
Later, for connecting these PHYs to MAC, the PHYs registered on the
mdiobus have to be referenced.

phy-handle
-----------
For each MAC node, a property "phy-handle" is used to reference the
PHY that is registered on an MDIO bus.

phy-mode
--------
Property "phy-mode" defines the type of PHY interface.

An example of this is shown below::

DSDT entry for MACs where PHY nodes are referenced
--------------------------------------------------
	Scope(\_SB.MCE0.PR17) // 1G
	{
	  Name (_DSD, Package () {
	     ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
		 Package () {
		     Package (2) {"phy-mode", "rgmii-id"},
		     Package (2) {"phy-handle", Package (){\_SB.MDI0.PHY1}}
	      }
	   })
	}

	Scope(\_SB.MCE0.PR18) // 1G
	{
	  Name (_DSD, Package () {
	    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
		Package () {
		    Package (2) {"phy-mode", "rgmii-id"},
		    Package (2) {"phy-handle", Package (){\_SB.MDI0.PHY2}}
	    }
	  })
	}

DSDT entry for MDIO node
------------------------
a) Silicon Component
--------------------
	Scope(_SB)
	{
	  Device(MDI0) {
	    Name(_HID, "NXP0006")
	    Name(_CCA, 1)
	    Name(_UID, 0)
	    Name(_CRS, ResourceTemplate() {
	      Memory32Fixed(ReadWrite, MDI0_BASE, MDI_LEN)
	      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared)
	       {
		 MDI0_IT
	       }
	    }) // end of _CRS for MDI0
	  } // end of MDI0
	}

b) Platform Component
---------------------
	Scope(\_SB.MDI0)
	{
	  Device(PHY1) {
	    Name (_ADR, 0x1)
	  } // end of PHY1

	  Device(PHY2) {
	    Name (_ADR, 0x2)
	  } // end of PHY2
	}
