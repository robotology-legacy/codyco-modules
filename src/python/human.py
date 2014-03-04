#!/xde
#
# Copyright (C) 2013-7 CODYCO Project
# Author:  Pauline Maurice
# email:  maurice@isir.upmc.fr
#
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#


# NOTE: this file can be used to import a XDE manikin in a script

import xdefw.human.knobycreator as knobycreator
import concepts.human

def addManikin(world, name, mass, height, pos, damping=1.):
	params = knobycreator.KnobyParams(mass, height)
	desc_creator = knobycreator.KnobyCreator(params)

	importer = concepts.human.HumanImporter(name, desc_creator.getHuman())

	importer.fillPhysicalLibrary(world)
	importer.addPhysicalInstance(world, name, pos, damping_scale=damping, weight_enabled=False, contact_material="material.gum")
	importer.fillGraphicalLibrary(world)
	importer.addGraphicalInstance(world, name, erase_root=True)


