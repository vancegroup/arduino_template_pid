Templated PID Library for Arduino
=================================
**Based on the Arduino PID Library - Version 1.0.1 - by Brett
Beauregard <br3ttb@gmail.com> <http://brettbeauregard.com>**

Modified into a template class and further tweaked by Ryan Pavlik
<rpavlik@iastate.edu> <http://academic.cleardefinition.com>

Usage
-----
Copy this whole directory (named `arduino_template_pid` into the
`Arduino/Libraries` folder or the `Libraries` folder in your sketchbook
directory.

After re-starting your Arduino IDE you should see this library listed.

Technical Details
-----------------
This is a template class, which means you might supply one or more parameters
when using it between angle brackets `<` `>`. Two parameters are possible: the first specifies the type of the input, output, and setpoint variables you'll use (something like `int` or `double` usually), and the second specifies the type of the tuning parameters you'll use (pretty much necessarily floating point, so something like `float` or `double`, which are equivalent on Arduino).

There are two default simple names (`typedefs`) for particular
variations that you can use instead of specifying those parameters
explicitly:

- `PIDd` ("double") has input, output, and setpoint variables as `double`, and the
   tuning variables as `double` as well.

- `PIDi` ("integer") has input, output, and setpoint variables as `int` (integers).

That covers the technical coding details, but not the technical controls
details. For an ultra-detailed explanation of why the code is the way it
is, please visit br3ttb's article series at
<http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/>

License
-------
GPL3

> Copyright Brett Beauregard 2008-2011.
>
> Copyright Iowa State University 2012.
>
> This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.
>
> This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
>
> You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
