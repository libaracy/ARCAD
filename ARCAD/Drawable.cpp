#include "stdafx.h"
#include "Drawable.h"


bool PolyBase::event(QEvent* e)
{
    if (e->type() == QEvent::DynamicPropertyChange)
    {
        auto ev = static_cast<QDynamicPropertyChangeEvent*>(e);

        auto prop = ev->propertyName();
        QVariant value = property(prop);

        int ver_index = prop.mid(1).toInt() - 1;
        int vec_index = prop[0] - 'x';

        if (ver_index >= 2 && vec_index >= 0 && vec_index <= 2)
        {
            if (value.isValid())
            {
                if (ver_index >= modify().vertexs.size())
                    modify().vertexs.resize(ver_index + 1);

                modify().vertexs[ver_index][vec_index] = value.toDouble();
            }
            else
            {
                modify().vertexs.resize(ver_index);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    return Drawable::event(e);
}