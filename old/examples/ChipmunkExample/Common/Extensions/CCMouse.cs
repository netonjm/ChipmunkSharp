
using ChipmunkSharp;
using CocosSharp;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CocosSharp
{

	public class CCMouse
	{


		public float DEFAULT_MOVE_MODIFICATOR = 4.5f;
		public float DEFAULT_ZOOM_SCALE = 0.3f;
		public bool IsDragBlocked { get; set; }

		public cpPivotJoint mouseJoint;
		public cpVect Position;
		public cpBody mouseBody;

		public CCPoint DragInitialPosition { get; set; }
		public CCNode DragContainer { get; set; }


		public CCPoint PositionPoint;
		public CCPoint PositionParentSpace;
		public CCPoint PositionLocationOnScreen;

		public DateTime lastTimeStamp = DateTime.Now;
		public bool dblclick;

		public bool rightclick;
		public bool leftclick;
		public bool midclick;

		public bool IsDragging { get { return DragContainer != null; } }


		public bool HasPosition
		{
			get
			{
				return Position != null;
			}
		}

		public bool HasBodyJoined { 
			get { 
				return mouseJoint != null; 
				}
		}

		public CCMouse()
		{
			mouseBody = cpBody.NewKinematic();
			Position = cpVect.Zero;
		}

		public void OnInterfaceButtonUp(float x, float y, bool left, bool right, bool middle, CCNode container)
		{
			//Touch doesn't have right button :)
			if (right)
				rightclick = false;
			if (left)
				leftclick = false;
			if (middle)
				midclick = false;

			dblclick = false;
		}

		public void OnInterfaceButtonDown(float x, float y, bool left, bool right, bool middle, CCNode container)
		{
			//Touch doesn't have right button :)
			if (right)
				rightclick = true;
			if (left)
				leftclick = true;
			if (middle)
				midclick = true;

			if (DateTime.Now.Subtract(lastTimeStamp).TotalMilliseconds < 200)
				dblclick = true;


			lastTimeStamp = DateTime.Now;
		}

		public void OnInterfaceMove(float x, float y, CCNode container)
		{
			var tmp_position = new CCPoint(x, y);
			UpdatePosition(tmp_position, container);
		}

		public void OnInterfaceMoveLocation(float x, float y, CCNode container)
		{
			var tmp_position = new CCPoint(x, y);
			UpdatePositionLocation(tmp_position, container);
		}

		public void OnInterfaceDragStart(CCNode container)
		{
			if (!HasBodyJoined && !IsDragBlocked)
			{
				DragContainer = container;
				DragInitialPosition = PositionPoint;
			}
		}

		public void OnInterfaceDragMove(float x, float y, CCNode container)
		{
			if (IsDragging && !HasBodyJoined && !IsDragBlocked)
			{
				CCPoint translation = new CCPoint(x, y) - DragInitialPosition;
				CCPoint newPos = container.Position + translation;
				container.Position = newPos;
			}
		}

		public void OnInterfaceEnded()
		{
			DragContainer = null;
			DragInitialPosition = CCPoint.Zero;
		}

		public void OnMouseMove(CCEventMouse mouseState, CCNode container)
		{
			OnMouseDown(mouseState, container);
			OnInterfaceDragMove(mouseState.CursorX, mouseState.CursorY, container);
		}

		public void OnMouseUp(CCEventMouse eventMouse, CCNode container)
		{
			OnInterfaceButtonUp(
				eventMouse.CursorX, eventMouse.CursorY,
				eventMouse.MouseButton == CCMouseButton.LeftButton,
				eventMouse.MouseButton == CCMouseButton.RightButton,
				eventMouse.MouseButton == CCMouseButton.MiddleButton,
				container
				);
		}

		public void OnMouseDown(CCEventMouse eventMouse, CCNode container)
		{
			OnInterfaceButtonDown(
				eventMouse.CursorX, eventMouse.CursorY,
				eventMouse.MouseButton == CCMouseButton.LeftButton,
				eventMouse.MouseButton == CCMouseButton.RightButton,
				eventMouse.MouseButton == CCMouseButton.MiddleButton,
				container
				);

			if (rightclick)
				OnInterfaceDragStart(container);
		}

		public void OnMouseUp(MouseState mouseState, CCNode container)
		{
			OnInterfaceButtonUp(
				mouseState.X, mouseState.Y,
				mouseState.RightButton == ButtonState.Pressed,
				mouseState.LeftButton == ButtonState.Pressed,
				mouseState.MiddleButton == ButtonState.Pressed,
				container
				);
		}
		public void OnMouseDown(MouseState mouseState, CCNode container)
		{
			OnInterfaceButtonDown(
				mouseState.X, mouseState.Y,
				mouseState.RightButton == ButtonState.Pressed,
				mouseState.LeftButton == ButtonState.Pressed,
				mouseState.MiddleButton == ButtonState.Pressed,
				container
				);

			if (rightclick)
				OnInterfaceDragStart(container);
		}



		public void OnMouseMove(MouseState mouseState, CCNode container)
		{
			OnMouseDown(mouseState, container);
			OnInterfaceDragMove(mouseState.X, mouseState.Y, container);
		}

		public void OnTouchMoved(CCTouch touch, CCNode container)
		{
			OnInterfaceMoveLocation(touch.LocationOnScreen.X, touch.LocationOnScreen.Y, container);
			OnInterfaceDragMove(touch.LocationOnScreen.X, touch.LocationOnScreen.Y, container);


#if WINDOWS_PHONE
			//Position = nodePosition.ToCpVect();
#endif


		}

		public void OnTouchesMoved(List<CCTouch> touches, CCNode dragContainer)
		{
			OnTouchMoved(touches.FirstOrDefault(), dragContainer);
		}

		public CCPoint ConvertTouchLocationToPoint(CCTouch touch, CCNode dragContainer)
		{
			var location = dragContainer.WorldToParentspace(touch.LocationOnScreen);
			location.Y = dragContainer.Window.WindowSizeInPixels.Height - location.Y;
			return location;
		}

		public CCPoint ConvertTouchLocationToPoint(List<CCTouch> touches, CCNode dragContainer)
		{
			return ConvertTouchLocationToPoint(touches.FirstOrDefault(), dragContainer);
		}

		public void OnTouchesEnded(List<CCTouch> touches, CCNode dragContainer)
		{
			//var location = dragContainer. ScreenToWorldspace(touch.LocationOnScreen);
			var location = ConvertTouchLocationToPoint(touches, dragContainer);

			OnInterfaceButtonUp(location.X, location.Y, true, false, false, dragContainer);
			OnInterfaceEnded();
			//#if WINDOWS_PHONE

			//			if (Position != null)
			//				Position = null;
			//#endif


		}

		public void OnTouchesCancelled(List<CCTouch> touches, CCNode container)
		{

			OnTouchesEnded(touches, container);
		}

		public void OnTouchesBegan(List<CCTouch> touches, CCNode container)
		{
			OnTouchBegan(touches.FirstOrDefault(), container);
		}

		public void OnTouchBegan(CCPoint location, CCNode container)
		{
			//UpdatePositionLocation(location, container);
			OnInterfaceButtonDown(Position.x, Position.y, true, false, false, container);
			//OnInterfaceDragStart(container);
		}

		public void OnTouchBegan(CCTouch touch, CCNode containter)
		{
			OnTouchBegan(touch.LocationOnScreen, containter);
		}


		#region UPDATE METHODS

		public void UpdatePositionLocation(CCPoint locationPosition, CCNode container)
		{

			PositionLocationOnScreen = new CCPoint(locationPosition);

			//locationPosition = container.WorldToParentspace(locationPosition); //We convert the corrdinates

			locationPosition.X *= container.ScaleX; //Scale the position to container scale
			locationPosition.Y *= container.ScaleY;

			PositionParentSpace = new CCPoint(locationPosition);

			locationPosition.Y = container.Window.WindowSizeInPixels.Height - locationPosition.Y; //Invert Y axis


			locationPosition -= container.Position; //Because the container can be dragged we need to rest the container actual position

			PositionPoint = locationPosition;

			Position = new cpVect(PositionPoint.X, PositionPoint.Y); //Save to general mouse position


		}

		public void UpdatePosition(float x, float y, CCNode container)
		{
			var locationPosition = new CCPoint(x, y);
			UpdatePositionLocation(locationPosition, container);
		}

		public void UpdatePosition(CCPoint cursorPosition, CCNode container)
		{
			UpdatePosition(cursorPosition.X, cursorPosition.Y, container);
		}



		#endregion


		#region Zoom scale

		public void ZoomIn(CCNode container)
		{
			Zoom(1f + DEFAULT_ZOOM_SCALE, container);
		}

		public void ZoomOut(CCNode container)
		{
			Zoom(1f - DEFAULT_ZOOM_SCALE, container);
		}

		public void Zoom(float scale, CCNode container)
		{
			container.ScaleX *= scale;
			container.ScaleY *= scale;
		}

		#endregion

		//public void OnMouseScroll(CCEventMouse mouseEvent, CCNode container)
		//{
		//	if (mouseEvent.ScrollY < 0)
		//		ZoomIn(container);

		//	if (mouseEvent.ScrollY > 0)
		//		ZoomOut(container);
		//}

		static CCMouse _instance { get; set; }
		public static CCMouse Instance
		{
			get
			{
				if (_instance == null) {

					_instance = new CCMouse();

				}
				return _instance;
			}

		}


		public void UpdateBodyPosition()
		{
			if (mouseBody != null && Position != null)
				mouseBody.SetPosition(Position);

		}

		public void UpdateBodyVelocity()
		{

			var bodyPosition = CCMouse.Instance.mouseBody.GetPosition();
			var newPoint = cpVect.cpvlerp(bodyPosition, Position, 0.25f);
			mouseBody.v = cpVect.cpvmult(cpVect.cpvsub(newPoint, bodyPosition), 60);


		}
	}

}
