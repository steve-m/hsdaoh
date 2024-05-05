/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 *
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HSDAOH_EXPORT_H
#define HSDAOH_EXPORT_H

#if defined __GNUC__
#  if __GNUC__ >= 4
#    define __HSDAOH_EXPORT   __attribute__((visibility("default")))
#    define __HSDAOH_IMPORT   __attribute__((visibility("default")))
#  else
#    define __HSDAOH_EXPORT
#    define __HSDAOH_IMPORT
#  endif
#elif _MSC_VER
#  define __HSDAOH_EXPORT     __declspec(dllexport)
#  define __HSDAOH_IMPORT     __declspec(dllimport)
#else
#  define __HSDAOH_EXPORT
#  define __HSDAOH_IMPORT
#endif

#ifndef hsdaoh_STATIC
#	ifdef hsdaoh_EXPORTS
#	define HSDAOH_API __HSDAOH_EXPORT
#	else
#	define HSDAOH_API __HSDAOH_IMPORT
#	endif
#else
#define HSDAOH_API
#endif
#endif /* HSDAOH_EXPORT_H */
