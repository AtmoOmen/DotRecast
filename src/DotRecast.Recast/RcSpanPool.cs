/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
DotRecast Copyright (c) 2023-2024 Choi Ikpil ikpil@naver.com

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;

namespace DotRecast.Recast
{
    /// A memory pool used for quick allocation of spans within a heightfield.
    /// Index 0 is reserved to mean "null"
    /// @see rcHeightfield
    public class RcSpanPool
    {
        private RcSpan[] storage = new RcSpan[64 * 1024];
        private uint firstUnalloc = 1;

        public RcSpanPool()
        {
            storage[0].next = firstUnalloc;
        }

        public ref RcSpan Span(uint index) => ref storage[index];

        public uint Alloc()
        {
            uint index = storage[0].next;
            if (index < firstUnalloc)
            {
                storage[0].next = storage[index].next;
                storage[index].next = 0;
                return index;
            }

            if (storage.Length == firstUnalloc)
            {
                var oldStorage = storage;
                storage = new RcSpan[oldStorage.Length * 2];
                Array.Copy(oldStorage, storage, oldStorage.Length);
            }

            storage[0].next = ++firstUnalloc;
            return index;
        }

        public void Free(uint index)
        {
            storage[index].next = storage[0].next;
            storage[0].next = index;
        }
    }
}
